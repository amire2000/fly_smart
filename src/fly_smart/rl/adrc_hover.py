"""Simple bounded ADRC controller for the four-action hover baseline."""

from dataclasses import dataclass
from math import pi

import numpy as np
import pybullet as p

from ..simulation.drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS, DroneModel, DroneState, PhysicsSettings
from ..simulation.drone_physics import PWM_MAX, PWM_MIN, PhysicsEngine


@dataclass(frozen=True)
class AdrcSettings:
    """Linear ESO and feedback settings for one scalar channel."""

    bandwidth: float
    b0: float
    output_limit: float = 1.0


class AdrcAxis:
    """Third-order linear ESO with bounded state-error feedback."""

    def __init__(self, settings: AdrcSettings) -> None:
        if settings.bandwidth <= 0.0 or settings.b0 <= 0.0 or settings.output_limit <= 0.0:
            raise ValueError("ADRC bandwidth, b0, and output_limit must be positive")
        self.settings = settings
        bandwidth = settings.bandwidth
        self.beta1 = 3.0 * bandwidth
        self.beta2 = 3.0 * bandwidth**2
        self.beta3 = bandwidth**3
        self.kp = bandwidth**2
        self.kd = 2.0 * bandwidth
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0
        self._last_output = 0.0

    def reset(self, measurement: float = 0.0) -> None:
        """Initialize the observer at a measured output with zero rate/disturbance."""
        self.z1 = measurement
        self.z2 = 0.0
        self.z3 = 0.0
        self._last_output = 0.0

    def update(self, target: float, measurement: float, dt: float) -> float:
        """Estimate disturbance and return one bounded control output."""
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        observer_error = self.z1 - measurement
        self.z1 += dt * (self.z2 - self.beta1 * observer_error)
        self.z2 += dt * (self.z3 + self.settings.b0 * self._last_output - self.beta2 * observer_error)
        self.z3 += dt * (-self.beta3 * observer_error)
        output = (self.kp * (target - self.z1) - self.kd * self.z2 - self.z3) / self.settings.b0
        self._last_output = float(np.clip(output, -self.settings.output_limit, self.settings.output_limit))
        return self._last_output


def _wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to the shortest signed distance around zero."""
    return (angle_rad + pi) % (2.0 * pi) - pi


class HoverAdrcController:
    """Map altitude and attitude errors to normalized throttle/torque actions."""

    def __init__(
        self,
        hover_altitude_m: float = 3.0,
        model: DroneModel = DEFAULT_DRONE_MODEL,
        settings: PhysicsSettings = DEFAULT_PHYSICS_SETTINGS,
        bandwidth: float = 6.0,
    ) -> None:
        if hover_altitude_m <= 0.0 or bandwidth <= 0.0:
            raise ValueError("hover_altitude_m and bandwidth must be positive")
        self.hover_altitude_m = hover_altitude_m
        self.settings = settings
        self.hover_action = self._pwm_to_action(
            PhysicsEngine(model, settings).pwm_from_thrust(model.mass_kg * -settings.gravity_z_mps2 / 4.0)
        )
        self._throttle = AdrcAxis(AdrcSettings(bandwidth, 2.0 * model.mass_kg * -settings.gravity_z_mps2, 0.8))
        self._roll = AdrcAxis(AdrcSettings(bandwidth, 0.05 / model.inertia_kg_m2[0]))
        self._pitch = AdrcAxis(AdrcSettings(bandwidth, 0.05 / model.inertia_kg_m2[1]))
        self._yaw = AdrcAxis(AdrcSettings(bandwidth, 0.02 / model.inertia_kg_m2[2]))

    def reset(self, state: DroneState) -> None:
        """Initialize all observers from the measured initial state."""
        roll, pitch, yaw = p.getEulerFromQuaternion(state.orientation_quaternion)
        self._throttle.reset(state.position_m[2])
        self._roll.reset(roll)
        self._pitch.reset(pitch)
        self._yaw.reset(yaw)

    def action(self, state: DroneState, dt: float | None = None) -> np.ndarray:
        """Return throttle, roll, pitch, and yaw actions in the RL action range."""
        dt = dt or 1.0 / self.settings.control_hz
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        roll, pitch, yaw = p.getEulerFromQuaternion(state.orientation_quaternion)
        throttle = self.hover_action + self._throttle.update(self.hover_altitude_m, state.position_m[2], dt)
        return np.asarray(
            (
                np.clip(throttle, -1.0, 1.0),
                self._roll.update(0.0, roll, dt),
                self._pitch.update(0.0, pitch, dt),
                self._yaw.update(0.0, _wrap_angle(yaw), dt),
            ),
            dtype=np.float32,
        )

    @staticmethod
    def _pwm_to_action(pwm: float) -> float:
        """Convert absolute PWM to the simulator's normalized collective action."""
        return 2.0 * (pwm - PWM_MIN) / (PWM_MAX - PWM_MIN) - 1.0
