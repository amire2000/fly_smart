"""Bounded four-action PID controller for the PyBullet hover baseline."""

from dataclasses import dataclass
from math import pi

import numpy as np
import pybullet as p

from ..simulation.drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS, DroneModel, DroneState, PhysicsSettings
from ..simulation.drone_physics import PWM_MAX, PWM_MIN, PhysicsEngine


@dataclass(frozen=True)
class PidGains:
    """PID coefficients for one normalized controller output."""

    kp: float
    ki: float
    kd: float


class _PidAxis:
    """One bounded PID accumulator with integral anti-windup."""

    def __init__(self, gains: PidGains, output_limit: float = 1.0) -> None:
        self.gains = gains
        self.output_limit = output_limit
        self.integral = 0.0

    def reset(self) -> None:
        """Clear accumulated integral error."""
        self.integral = 0.0

    def update(self, error: float, derivative: float, dt: float) -> float:
        """Return one bounded PID output and avoid integrating into saturation."""
        proposed_integral = self.integral + error * dt
        raw = self.gains.kp * error + self.gains.ki * proposed_integral + self.gains.kd * derivative
        output = float(np.clip(raw, -self.output_limit, self.output_limit))
        if abs(raw) <= self.output_limit or np.sign(error) != np.sign(raw - output):
            self.integral = proposed_integral
        return output


def _wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to the shortest signed distance around zero."""
    return (angle_rad + pi) % (2.0 * pi) - pi


class HoverPidController:
    """Map altitude and attitude errors to the four normalized RL actions."""

    def __init__(
        self,
        hover_altitude_m: float = 3.0,
        model: DroneModel = DEFAULT_DRONE_MODEL,
        settings: PhysicsSettings = DEFAULT_PHYSICS_SETTINGS,
        throttle_gains: PidGains = PidGains(0.8, 0.25, 0.35),
        roll_gains: PidGains = PidGains(1.4, 0.05, 0.25),
        pitch_gains: PidGains = PidGains(1.4, 0.05, 0.25),
        yaw_gains: PidGains = PidGains(0.8, 0.02, 0.15),
    ) -> None:
        if hover_altitude_m <= 0.0:
            raise ValueError("hover_altitude_m must be positive")
        self.hover_altitude_m = hover_altitude_m
        self.settings = settings
        self.hover_action = self._pwm_to_action(
            PhysicsEngine(model, settings).pwm_from_thrust(model.mass_kg * -settings.gravity_z_mps2 / 4.0)
        )
        self._throttle = _PidAxis(throttle_gains, 0.8)
        self._roll = _PidAxis(roll_gains, 1.0)
        self._pitch = _PidAxis(pitch_gains, 1.0)
        self._yaw = _PidAxis(yaw_gains, 1.0)

    def reset(self) -> None:
        """Reset all controller integrators before a new hover attempt."""
        for axis in (self._throttle, self._roll, self._pitch, self._yaw):
            axis.reset()

    def action(self, state: DroneState, dt: float | None = None) -> np.ndarray:
        """Return throttle, roll, pitch, and yaw actions in the RL action range."""
        dt = dt or 1.0 / self.settings.control_hz
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        roll, pitch, yaw = p.getEulerFromQuaternion(state.orientation_quaternion)
        roll_rate, pitch_rate, yaw_rate = state.angular_velocity_body_rad_s
        throttle = self.hover_action + self._throttle.update(
            self.hover_altitude_m - state.position_m[2], -state.linear_velocity_mps[2], dt
        )
        return np.asarray(
            (
                np.clip(throttle, -1.0, 1.0),
                self._roll.update(-roll, -roll_rate, dt),
                self._pitch.update(-pitch, -pitch_rate, dt),
                self._yaw.update(_wrap_angle(-yaw), -yaw_rate, dt),
            ),
            dtype=np.float32,
        )

    @staticmethod
    def _pwm_to_action(pwm: float) -> float:
        """Convert absolute PWM to the simulator's normalized collective action."""
        return 2.0 * (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN) - 1.0
