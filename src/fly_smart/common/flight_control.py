"""Reusable attitude and vertical-speed feedback controllers."""

from math import cos, isfinite, pi

from ..mission import MissionConfig

from .pid import PID


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the range from negative pi to positive pi radians."""
    return (angle + pi) % (2 * pi) - pi


class AttitudeController:
    """Turn roll, pitch, and yaw targets into body-torque requests."""

    def __init__(self, pitch_gains: tuple[float, float, float] | None = None, roll_gains: tuple[float, float, float] | None = None) -> None:
        """Create roll/yaw controllers and optional roll/pitch-specific tuning."""
        self.roll_pid = PID(*(roll_gains or (0.002, 0.0, 0.001)))
        self.pitch_pid = PID(*(pitch_gains or (0.002, 0.0, 0.001)))
        self.yaw_pid = PID(0.001, 0.0, 0.0005)

    def reset(self) -> None:
        """Clear all PID history after a reset or controller-mode change."""
        for controller in (self.roll_pid, self.pitch_pid, self.yaw_pid):
            controller.reset()

    def update(self, imu: object, yaw_target: float, roll_target: float = 0.0, pitch_target: float = 0.0, dt: float = 1 / 120) -> tuple[float, float, float]:
        """Return roll, pitch, and yaw torque corrections from ideal IMU data."""
        roll, pitch, yaw = imu.roll_pitch_yaw_rad
        roll_rate, pitch_rate, yaw_rate = imu.angular_velocity_body_rad_s
        return (
            self.roll_pid.update(roll_target - roll, roll_rate, dt),
            self.pitch_pid.update(pitch_target - pitch, pitch_rate, dt),
            self.yaw_pid.update(wrap_angle(yaw_target - yaw), yaw_rate, dt),
        )


class VerticalSpeedController:
    """Shared PI feedback with tilt compensation and saturation anti-windup."""

    def __init__(self, config: MissionConfig, max_thrust_n: float) -> None:
        if not isfinite(max_thrust_n) or max_thrust_n <= 0:
            raise ValueError("max_thrust_n must be positive and finite")
        self.hover_thrust_n = config.hover_thrust_n
        self.max_thrust_n = max_thrust_n
        self.pid = PID(*config.vertical_velocity_pid_gains, integral_limit=config.vertical_velocity_integral_limit)
        self.saturated = False

    def initialize(self, thrust_n: float, measured_vz: float, target_vz: float, tilt: float) -> None:
        """Seed integral from the preceding force for a bumpless takeoff handoff."""
        if self.pid.ki:
            correction = thrust_n * tilt - self.hover_thrust_n - self.pid.kp * (target_vz - measured_vz)
            self.pid.integral = max(-self.pid.integral_limit, min(self.pid.integral_limit, correction / self.pid.ki))

    def update(self, target_vz: float, measured_vz: float, roll: float, pitch: float, dt: float) -> float:
        """Return bounded collective; freeze integration only when it worsens saturation."""
        tilt = max(cos(roll) * cos(pitch), 0.5)
        error = target_vz - measured_vz
        old_integral = self.pid.integral
        force = (self.hover_thrust_n + self.pid.update(error, 0.0, dt)) / tilt
        self.saturated = force < 0 or force > self.max_thrust_n
        if (force > self.max_thrust_n and error > 0) or (force < 0 and error < 0):
            self.pid.integral = old_integral
            force = (self.hover_thrust_n + self.pid.kp * error + self.pid.ki * old_integral) / tilt
        return max(0.0, min(self.max_thrust_n, force))
