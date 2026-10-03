"""Observation, reward, and safety rules for the state-based hover task."""

from math import isfinite

import numpy as np
import pybullet as p

from ..simulation.drone_model import DroneState, PhysicsStep


class HoverTask:
    """Build normalized hover observations and evaluate one transition."""

    def __init__(self, hover_altitude_m: float = 3.0) -> None:
        self.hover_altitude_m = hover_altitude_m

    def observation(self, state: DroneState, physics: PhysicsStep | None) -> np.ndarray:
        """Return altitude, motion, attitude, rate, actuator, and battery features."""
        roll, pitch, yaw = p.getEulerFromQuaternion(state.orientation_quaternion)
        motor_rpms = physics.motor_rpms if physics else (0.0,) * 4
        soc = physics.battery_state_of_charge if physics else 1.0
        values = (
            (state.position_m[2] - self.hover_altitude_m) / 3.0,
            *(component / 10.0 for component in state.linear_velocity_mps),
            roll / np.pi,
            pitch / np.pi,
            yaw / np.pi,
            *(rate / 10.0 for rate in state.angular_velocity_body_rad_s),
            *(rpm / 10000.0 for rpm in motor_rpms),
            soc,
        )
        return np.clip(np.asarray(values, dtype=np.float32), -1.0, 1.0)

    def reward(self, state: DroneState, action: np.ndarray) -> float:
        """Return a dense level-hover reward with a small action penalty."""
        roll, pitch, yaw = p.getEulerFromQuaternion(state.orientation_quaternion)
        altitude_error = (state.position_m[2] - self.hover_altitude_m) / 3.0
        velocity_error = np.asarray(state.linear_velocity_mps, dtype=np.float32) / 10.0
        attitude_error = np.asarray((roll, pitch, yaw), dtype=np.float32) / np.pi
        rates = np.asarray(state.angular_velocity_body_rad_s, dtype=np.float32) / 10.0
        return float(1.0 - 1.5 * altitude_error**2 - 0.5 * np.dot(velocity_error, velocity_error) - np.dot(attitude_error, attitude_error) - 0.25 * np.dot(rates, rates) - 0.01 * np.dot(action, action))

    def terminated(self, state: DroneState) -> tuple[bool, str | None]:
        """Return whether the physical state crossed a safety boundary."""
        roll, pitch, _ = p.getEulerFromQuaternion(state.orientation_quaternion)
        values = (*state.position_m, *state.linear_velocity_mps, roll, pitch, *state.angular_velocity_body_rad_s)
        if not all(isfinite(value) for value in values):
            return True, "non-finite state"
        if state.position_m[2] < 0.2 or state.position_m[2] > 8.0:
            return True, "altitude limit"
        if abs(roll) > 1.2 or abs(pitch) > 1.2:
            return True, "tilt limit"
        return False, None
