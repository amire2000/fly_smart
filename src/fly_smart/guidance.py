"""Sensor-only DY/TTC guidance state machine and bounded attitude requests."""

from dataclasses import dataclass
from enum import Enum
from math import cos, isfinite, radians

from .common.flight_control import VerticalSpeedController
from .common.pid import PID
from .mission import MissionConfig
from .sensing import BarometerReading
from .trajectory import TrajectoryCommand, VerticalTargetPlanner
from .visual_guidance import VisualEstimate


class FlightPhase(str, Enum):
    """Mission phases; tracking submodes are separate from recovery phases."""

    TAKEOFF = "takeoff"
    ACQUIRE = "acquire"
    TRACK = "track"
    RECOVER = "recover"
    COMMIT = "commit"
    ABORT = "abort"


class VerticalControlMode(str, Enum):
    """Source of the active vertical-speed reference."""

    HOLD = "hold"
    DY = "dy"
    BLEND = "blend"
    TTC = "ttc"
    COMMIT = "commit"
    ABORT = "abort"


@dataclass(frozen=True)
class GuidanceInput:
    """Control-time fused vertical state and capture-time visual estimate."""

    now_s: float
    barometer: BarometerReading
    visual: VisualEstimate | None = None
    attitude_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class GuidanceCommand:
    """Bounded collective/RPY targets plus controller diagnostics."""

    phase: FlightPhase
    thrust_n: float
    pitch_target_rad: float
    trajectory: TrajectoryCommand | None
    reset_ttc: bool = False
    commit_expired: bool = False
    vertical_velocity_target_mps: float | None = None
    vertical_control_mode: VerticalControlMode = VerticalControlMode.HOLD
    camera_dy_correction_mps: float = 0.0
    path_slope_target_m_per_m: float | None = None
    roll_target_rad: float = 0.0
    yaw_target_rad: float = 0.0
    ttc_weight: float = 0.0
    requested_vertical_velocity_mps: float = 0.0
    reason: str = ""
    thrust_saturated: bool = False
    frame_age_s: float | None = None
    visual_reason: str = ""
    bearing_alpha_deg: float | None = None
    bearing_beta_rad: float | None = None


class StrikeGuidance:
    """Compose planning and feedback while owning mission states and deadlines."""

    def __init__(self, config: MissionConfig, max_thrust_n: float | None = None) -> None:
        self.config = config
        self.max_thrust_n = max_thrust_n if max_thrust_n is not None else config.hover_thrust_n * 4
        self.phase = FlightPhase.TAKEOFF
        self.planner = VerticalTargetPlanner(config)
        self.altitude_pid = PID(*config.altitude_pid_gains, integral_limit=config.altitude_integral_limit)
        self.vertical_control = VerticalSpeedController(config, self.max_thrust_n)
        self.lateral_pd = PID(*config.lateral_pd_gains)
        self.last_command = GuidanceCommand(self.phase, config.hover_thrust_n, 0.0, None)
        self.last_time_s: float | None = None
        self.phase_since_s = 0.0
        self.bearing_since_s: float | None = None
        self.ttc_since_s: float | None = None
        self.ttc_active = False
        self.last_bearing_s: float | None = None
        self.hold_altitude_m = config.takeoff_altitude_m
        self.heading_rad: float | None = None
        self.terminal_command: GuidanceCommand | None = None
        self.commit_deadline_s: float | None = None
        self.recovery_reason = ""

    def _enter(self, phase: FlightPhase, now_s: float) -> None:
        self.phase = phase
        self.phase_since_s = now_s
        self.bearing_since_s = None
        self.ttc_since_s = None
        self.ttc_active = False
        self.planner.weight = 0.0

    def update(self, data: GuidanceInput) -> GuidanceCommand:
        """Advance one tick; freshness and deadlines always use capture timestamps."""
        c = self.config
        dt = data.now_s - self.last_time_s if self.last_time_s is not None else 1 / 120
        valid = len(data.attitude_rad) == 3 and all(isfinite(v) for v in (data.now_s, data.barometer.altitude_m,
                    data.barometer.vertical_velocity_mps, *data.attitude_rad)) and dt > 0
        self.last_time_s = data.now_s
        if not valid:
            self.phase = FlightPhase.ABORT
            self.last_command = GuidanceCommand(self.phase, min(c.hover_thrust_n, self.max_thrust_n), 0.0, None,
                vertical_control_mode=VerticalControlMode.ABORT, reason="invalid_sensor_or_time")
            return self.last_command
        if self.heading_rad is None:
            self.heading_rad = data.attitude_rad[2]
        z, vz = data.barometer.altitude_m, data.barometer.vertical_velocity_mps
        roll, pitch, _ = data.attitude_rad
        visual = data.visual
        bearing_ok = bool(visual and visual.bearing_valid and
            0 <= data.now_s - visual.captured_at_s <= c.frame_max_age_s and
            all(isfinite(v) for v in (visual.alpha_deg, visual.beta_rad, visual.alpha_rate_deg_s, visual.beta_rate_rad_s)))
        if bearing_ok:
            self.last_bearing_s = visual.captured_at_s
            if self.bearing_since_s is None:
                self.bearing_since_s = data.now_s
        else:
            self.bearing_since_s = None
        tau = None
        if bearing_ok and visual.ttc_reliable and visual.observation:
            remaining = visual.observation.ttc_s - (data.now_s - visual.captured_at_s)
            if isfinite(remaining) and remaining > 0:
                tau = remaining
        feasible = self.planner.feasible(z, vz, tau if tau is not None and tau <= c.ttc_activation_s else None)
        reset_ttc = False
        if self.phase == FlightPhase.TAKEOFF:
            thrust = c.hover_thrust_n + self.altitude_pid.update(c.takeoff_altitude_m - z, vz, dt)
            if vz >= c.takeoff_max_climb_velocity_mps:
                thrust = min(thrust, c.hover_thrust_n)
            thrust = max(0.0, min(self.max_thrust_n, thrust))
            if abs(z - c.takeoff_altitude_m) <= c.takeoff_altitude_tolerance_m and abs(vz) <= c.takeoff_velocity_tolerance_mps:
                self._enter(FlightPhase.ACQUIRE, data.now_s)
                self.planner.initialize(vz)
                self.vertical_control.initialize(thrust, vz, vz, max(cos(roll) * cos(pitch), 0.5))
                reset_ttc = True
            else:
                self.last_command = GuidanceCommand(self.phase, thrust, 0.0, None, yaw_target_rad=self.heading_rad)
                return self.last_command

        if self.phase == FlightPhase.ACQUIRE:
            if data.now_s - self.phase_since_s >= c.acquire_timeout_s:
                self._enter(FlightPhase.ABORT, data.now_s)
            elif self.bearing_since_s is not None and data.now_s - self.bearing_since_s >= c.bearing_dwell_s:
                self._enter(FlightPhase.TRACK, data.now_s)

        if self.phase == FlightPhase.TRACK and (not bearing_ok or not feasible):
            if not bearing_ok and self.terminal_command is not None and self.commit_deadline_s is not None and data.now_s < self.commit_deadline_s:
                self._enter(FlightPhase.COMMIT, data.now_s)
            else:
                self._enter(FlightPhase.RECOVER, data.now_s)
                self.recovery_reason = "descent_infeasible" if not feasible else "bearing_unusable"
                if not bearing_ok and self.last_bearing_s is not None:
                    self.phase_since_s = self.last_bearing_s
                self.hold_altitude_m = z
                self.terminal_command = None

        if self.phase == FlightPhase.RECOVER:
            if data.now_s - self.phase_since_s >= c.recovery_timeout_s:
                self._enter(FlightPhase.ABORT, data.now_s)
            elif feasible and self.bearing_since_s is not None and data.now_s - self.bearing_since_s >= c.bearing_dwell_s:
                self._enter(FlightPhase.TRACK, data.now_s)

        recovery_reason = self.recovery_reason if self.phase == FlightPhase.ABORT else ""
        expired = self.phase == FlightPhase.COMMIT and data.now_s >= self.commit_deadline_s
        if expired:
            self._enter(FlightPhase.ABORT, data.now_s)
        mode = VerticalControlMode.HOLD
        reason = ""
        desired_pitch = desired_roll = 0.0
        if self.phase == FlightPhase.TRACK:
            threshold = c.ttc_exit_s if self.ttc_active else c.ttc_activation_s
            eligible = tau is not None and tau <= threshold
            if eligible:
                if self.ttc_since_s is None:
                    self.ttc_since_s = data.now_s
                if data.now_s - self.ttc_since_s >= c.ttc_dwell_s:
                    self.ttc_active = True
            else:
                self.ttc_since_s = None
                self.ttc_active = False
            target_vz = self.planner.command(z, visual.alpha_deg, visual.alpha_rate_deg_s,
                                            tau if self.ttc_active else None, dt)
            mode = VerticalControlMode.TTC if self.planner.weight >= 1 else (VerticalControlMode.BLEND if self.planner.weight > 0 else VerticalControlMode.DY)
            desired_pitch = min(c.max_pitch_rad, radians(c.approach_pitch_deg))
            desired_roll = self.lateral_pd.update(-visual.beta_rad, visual.beta_rate_rad_s, dt)
            desired_roll = max(-radians(c.max_roll_deg), min(radians(c.max_roll_deg), desired_roll))
            reason = visual.reason
        elif self.phase == FlightPhase.COMMIT:
            saved = self.terminal_command
            target_vz = saved.vertical_velocity_target_mps
            desired_pitch, desired_roll = saved.pitch_target_rad, saved.roll_target_rad
            mode = VerticalControlMode.COMMIT
            reason = "bounded_blind_commit"
        else:
            if self.phase == FlightPhase.RECOVER:
                loss_age = data.now_s - self.last_bearing_s if self.last_bearing_s is not None else c.recovery_grace_s
                if self.recovery_reason == "bearing_unusable" and loss_age < c.recovery_grace_s:
                    requested = self.planner.velocity_mps
                    desired_pitch, desired_roll = self.last_command.pitch_target_rad, self.last_command.roll_target_rad
                else:
                    requested = c.vertical_position_correction * (self.hold_altitude_m - z)
                    requested = max(-c.max_vertical_position_correction_mps, min(c.max_vertical_position_correction_mps, requested))
                reason = self.recovery_reason
            else:
                requested = c.vertical_position_correction * ((c.takeoff_altitude_m if self.phase == FlightPhase.ACQUIRE else z) - z)
                reason = "acquiring_bearing" if self.phase == FlightPhase.ACQUIRE else (recovery_reason or "mission_aborted")
                requested = max(-c.max_vertical_position_correction_mps, min(c.max_vertical_position_correction_mps, requested))
            target_vz = self.planner.slew(requested, dt)
            if self.phase == FlightPhase.ABORT:
                mode = VerticalControlMode.ABORT
        pitch_target = self._slew(self.last_command.pitch_target_rad, desired_pitch, radians(c.approach_pitch_slew_deg_s) * dt)
        roll_target = self._slew(self.last_command.roll_target_rad, desired_roll, radians(c.roll_slew_deg_s) * dt)
        thrust = self.vertical_control.update(target_vz, vz, roll, pitch, dt)
        altitude_target = c.impact_altitude_m if self.phase in (FlightPhase.TRACK, FlightPhase.COMMIT) else (c.takeoff_altitude_m if self.phase == FlightPhase.ACQUIRE else (self.hold_altitude_m if self.phase == FlightPhase.RECOVER else z))
        trajectory = self.terminal_command.trajectory if self.phase == FlightPhase.COMMIT else TrajectoryCommand(None, self.planner.requested_mps, altitude_target)
        command = GuidanceCommand(self.phase, thrust, pitch_target,
            trajectory,
            reset_ttc=reset_ttc, commit_expired=expired, vertical_velocity_target_mps=target_vz,
            vertical_control_mode=mode, camera_dy_correction_mps=self.planner.dy_mps,
            roll_target_rad=roll_target, yaw_target_rad=self.heading_rad,
            ttc_weight=self.planner.weight, requested_vertical_velocity_mps=self.planner.requested_mps,
            reason=reason, thrust_saturated=self.vertical_control.saturated,
            frame_age_s=data.now_s - visual.captured_at_s if visual else None,
            visual_reason=(visual.reason if bearing_ok else ("frame_stale" if visual and visual.bearing_valid else (visual.reason if visual else "no_frame"))),
            bearing_alpha_deg=visual.alpha_deg if bearing_ok else None,
            bearing_beta_rad=visual.beta_rad if bearing_ok else None)
        if self.phase == FlightPhase.TRACK:
            if tau is not None and tau <= c.blind_commit_max_s and feasible and abs(visual.alpha_deg) <= c.terminal_bearing_tolerance_deg and abs(visual.beta_rad) <= radians(c.terminal_bearing_tolerance_deg):
                self.terminal_command = command
                self.commit_deadline_s = min(visual.captured_at_s + visual.observation.ttc_s, visual.captured_at_s + c.blind_commit_max_s)
            else:
                self.terminal_command = None
                self.commit_deadline_s = None
        self.last_command = command
        return command

    @staticmethod
    def _slew(previous: float, requested: float, step: float) -> float:
        return previous + max(-step, min(step, requested - previous))
