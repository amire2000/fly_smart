"""Configuration shared by real-flight adapters and the simulator."""

from dataclasses import dataclass, fields
from math import isfinite, radians


@dataclass(frozen=True)
class MissionConfig:
    """Mission, camera, estimator, and controller settings independent of a world model."""

    vehicle_mass_kg: float = 0.65
    gravity_mps2: float = 9.81
    camera_width_px: int = 640
    camera_height_px: int = 480
    camera_hz: int = 30
    camera_fov_deg: float = 90.0
    camera_look_down_deg: float = 0.0
    takeoff_altitude_m: float = 15.0
    takeoff_max_climb_velocity_mps: float = 7.2
    impact_altitude_m: float = 1.0
    forward_speed_mps: float = 13.0
    nominal_pitch_deg: float = 20.0
    max_pitch_deg: float = 20.0
    max_descent_velocity_mps: float = 5.5
    max_climb_velocity_mps: float = 3.0
    min_ttc_s: float = 0.2
    ttc_activation_s: float = 8.0
    commit_box_height_fraction: float = 0.1
    ttc_alpha: float = 0.85
    ttc_beta: float = 0.20
    min_growth_px_per_s: float = 0.01
    ttc_unavailable_descent_velocity_mps: float = 1.5
    ttc_unavailable_descent_slope_m_per_m: float = 0.50
    ttc_unavailable_pitch_boost_deg: float = 0.0
    camera_dy_gain_mps_per_deg: float = 0.20
    camera_dy_deadband_deg: float = 1.0
    camera_dy_max_correction_mps: float = 2.0
    vertical_estimator_alpha: float = 0.08
    vertical_estimator_beta: float = 0.005
    altitude_pid_gains: tuple[float, float, float] = (1.8, 0.05, 2.2)
    altitude_integral_limit: float = 0.5
    forward_speed_pid_gains: tuple[float, float, float] = (0.03, 0.0, 0.002)
    forward_pitch_integral_limit: float = 0.2
    roll_attitude_pid_gains: tuple[float, float, float] = (0.02, 0.0, 0.01)
    pitch_attitude_pid_gains: tuple[float, float, float] = (0.008, 0.0, 0.006)
    vertical_velocity_pid_gains: tuple[float, float, float] = (1.0, 0.4, 0.0)
    vertical_velocity_integral_limit: float = 5.0
    approach_pitch_deg: float = 7.0
    approach_pitch_slew_deg_s: float = 5.0
    camera_dy_rate_gain: float = 0.02
    bearing_filter_time_s: float = 0.15
    lateral_pd_gains: tuple[float, float, float] = (0.8, 0.0, 4.0)
    max_roll_deg: float = 15.0
    roll_slew_deg_s: float = 15.0
    frame_max_age_s: float = 0.1
    bearing_dwell_s: float = 0.1
    acquire_timeout_s: float = 5.0
    recovery_grace_s: float = 0.2
    recovery_timeout_s: float = 1.0
    ttc_window_s: float = 0.5
    ttc_min_samples: int = 8
    ttc_fit_max_relative_rms: float = 0.15
    ttc_dwell_s: float = 0.3
    ttc_blend_s: float = 0.5
    ttc_exit_s: float = 10.0
    vertical_slew_mps2: float = 1.0
    braking_delay_s: float = 0.2
    terminal_altitude_tolerance_m: float = 0.2
    terminal_bearing_tolerance_deg: float = 2.0
    blind_commit_max_s: float = 0.3
    vertical_position_correction: float = 0.8
    max_vertical_position_correction_mps: float = 2.0
    takeoff_altitude_tolerance_m: float = 0.2
    takeoff_velocity_tolerance_mps: float = 0.5
    commit_timeout_margin_s: float = 5.0

    def __post_init__(self) -> None:
        """Reject invalid physical/controller tuning at the configuration seam."""
        for field in fields(self):
            value = getattr(self, field.name)
            values = value if isinstance(value, tuple) else (value,)
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not isfinite(v) for v in values):
                raise ValueError(f"{field.name} must contain finite numbers")
            if isinstance(value, tuple) and len(value) != 3:
                raise ValueError(f"{field.name} must contain three gains")
        for name in ("camera_width_px", "camera_height_px", "camera_hz", "ttc_min_samples"):
            value = getattr(self, name)
            if not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        for name in ("vehicle_mass_kg", "gravity_mps2", "bearing_filter_time_s", "frame_max_age_s",
                     "ttc_window_s", "ttc_blend_s", "vertical_slew_mps2", "max_descent_velocity_mps",
                     "acquire_timeout_s", "recovery_timeout_s"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if not 0 < self.camera_fov_deg < 180:
            raise ValueError("camera_fov_deg must be between 0 and 180")
        if self.ttc_min_samples < 2 or self.ttc_exit_s < self.ttc_activation_s:
            raise ValueError("TTC requires >=2 samples and exit >= activation")
        if not 0 <= self.recovery_grace_s < self.recovery_timeout_s:
            raise ValueError("recovery grace must be shorter than timeout")
        if self.takeoff_altitude_m <= self.impact_altitude_m:
            raise ValueError("takeoff altitude must exceed target altitude")

    @property
    def hover_thrust_n(self) -> float:
        return self.vehicle_mass_kg * self.gravity_mps2

    @property
    def nominal_pitch_rad(self) -> float:
        return radians(self.nominal_pitch_deg)

    @property
    def max_pitch_rad(self) -> float:
        return radians(self.max_pitch_deg)

    @property
    def ttc_unavailable_pitch_boost_rad(self) -> float:
        return radians(self.ttc_unavailable_pitch_boost_deg)

    @property
    def commit_box_height_px(self) -> float:
        return self.camera_height_px * self.commit_box_height_fraction
