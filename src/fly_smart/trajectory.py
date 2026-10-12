"""TTC-synchronised altitude planning without metric target range."""

from dataclasses import dataclass
from math import sqrt

from .mission import MissionConfig


@dataclass(frozen=True)
class TrajectoryCommand:
    """Vertical plan; forward velocity is absent for bearing-only approach."""

    forward_velocity_mps: float | None
    vertical_velocity_mps: float
    altitude_target_m: float


class TtcDescentPlanner:
    """Use TTC as time-to-go for reaching the known impact altitude."""

    def __init__(self, config: MissionConfig) -> None:
        self.config = config

    def command(self, ttc_s: float | None, current_altitude_m: float) -> TrajectoryCommand:
        """Return nominal forward motion and a TTC-synchronised vertical target.

        Target size and distance are intentionally absent. TTC supplies only
        the available time; the barometer supplies current altitude and the
        mission supplies the altitude at which contact should occur.
        """
        altitude_error_m = self.config.impact_altitude_m - current_altitude_m
        if ttc_s is None:
            vertical_velocity_mps = 0.0
            altitude_target_m = self.config.impact_altitude_m
        else:
            time_to_go_s = max(ttc_s, self.config.min_ttc_s)
            vertical_velocity_mps = altitude_error_m / time_to_go_s
            altitude_target_m = self.config.impact_altitude_m
        vertical_velocity_mps = max(
            -self.config.max_descent_velocity_mps,
            min(self.config.max_climb_velocity_mps, vertical_velocity_mps),
        )
        return TrajectoryCommand(
            self.config.forward_speed_mps,
            vertical_velocity_mps,
            altitude_target_m,
        )


class VerticalTargetPlanner:
    """Own DY PD, TTC blending, terminal braking and a continuous speed reference."""

    def __init__(self, config: MissionConfig) -> None:
        self.config = config
        self.ttc = TtcDescentPlanner(config)
        self.velocity_mps = 0.0
        self.weight = 0.0
        self.requested_mps = 0.0
        self.dy_mps = 0.0

    def initialize(self, measured_vz: float) -> None:
        self.velocity_mps = measured_vz
        self.weight = 0.0

    def command(self, altitude: float, alpha_deg: float, alpha_rate_deg_s: float,
                ttc_s: float | None, dt: float) -> float:
        """Select and blend planning inputs, then apply braking and reference slew."""
        c = self.config
        error = max(0.0, alpha_deg - c.camera_dy_deadband_deg)
        self.dy_mps = max(-c.camera_dy_max_correction_mps, min(0.0,
            -c.camera_dy_gain_mps_per_deg * error - c.camera_dy_rate_gain * alpha_rate_deg_s)) if error else 0.0
        self.weight = min(1.0, self.weight + dt / c.ttc_blend_s) if ttc_s is not None else 0.0
        candidate = min(self.dy_mps, self.ttc.command(ttc_s, altitude).vertical_velocity_mps) if ttc_s is not None else self.dy_mps
        requested = (1 - self.weight) * self.dy_mps + self.weight * candidate
        h = max(0.0, altitude - c.impact_altitude_m)
        if h <= c.terminal_altitude_tolerance_m:
            requested = c.vertical_position_correction * (c.impact_altitude_m - altitude)
            requested = max(-c.max_vertical_position_correction_mps, min(c.max_vertical_position_correction_mps, requested))
        else:
            a, delay = c.vertical_slew_mps2, c.braking_delay_s
            safe_down = sqrt((a * delay) ** 2 + 2 * a * h) - a * delay
            requested = max(requested, -safe_down)
        return self.slew(requested, dt)

    def slew(self, requested: float, dt: float) -> float:
        """Apply the same speed/acceleration limits in tracking and recovery."""
        c = self.config
        self.requested_mps = max(-c.max_descent_velocity_mps, min(c.max_climb_velocity_mps, requested))
        step = c.vertical_slew_mps2 * dt
        self.velocity_mps += max(-step, min(step, self.requested_mps - self.velocity_mps))
        return self.velocity_mps

    def feasible(self, altitude: float, measured_vz: float, ttc_s: float | None) -> bool:
        """Screen acceleration/cruise/braking time; no hardware authority guarantee."""
        c = self.config
        delta = altitude - c.impact_altitude_m
        h = abs(delta)
        a = c.vertical_slew_mps2
        limit = c.max_descent_velocity_mps if delta >= 0 else c.max_climb_velocity_mps
        v = -measured_vz if delta >= 0 else measured_vz
        if v > limit:
            return False
        downward = max(0.0, -measured_vz)
        if downward ** 2 / (2 * a) + downward * c.braking_delay_s > max(0.0, delta) + c.terminal_altitude_tolerance_m:
            return False
        if delta < 0 and max(0.0, v) ** 2 / (2 * a) + max(0.0, v) * c.braking_delay_s > h + c.terminal_altitude_tolerance_m:
            return False
        if h <= c.terminal_altitude_tolerance_m and abs(measured_vz) <= c.takeoff_velocity_tolerance_mps:
            return True
        if ttc_s is None:
            return True
        peak = sqrt(a * h + v * v / 2)
        if peak <= limit:
            duration = (2 * peak - v) / a
        else:
            distance = h - (limit * limit - v * v) / (2 * a) - limit * limit / (2 * a)
            duration = (2 * limit - v) / a + max(0.0, distance) / limit
        return duration + c.braking_delay_s <= ttc_s
