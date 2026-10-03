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
        """Return altitude, motion, attitude, and body-rate features."""
        del physics  # Actuator telemetry is not part of the hover policy state.
        roll, pitch, yaw = p.getEulerFromQuaternion(state.orientation_quaternion)
        values = (
            (state.position_m[2] - self.hover_altitude_m) / 3.0,
            *(component / 10.0 for component in state.linear_velocity_mps),
            roll / np.pi,
            pitch / np.pi,
            yaw / np.pi,
            *(rate / 10.0 for rate in state.angular_velocity_body_rad_s),
        )
        return np.clip(np.asarray(values, dtype=np.float32), -1.0, 1.0)

    def reward(self, state: DroneState, action: np.ndarray, terminated: bool = False) -> float:
        """Return a dense hover reward with strong altitude and failure costs."""
        roll, pitch, yaw = p.getEulerFromQuaternion(state.orientation_quaternion)
        altitude_error = (state.position_m[2] - self.hover_altitude_m) / 3.0
        velocity_error = np.asarray(state.linear_velocity_mps, dtype=np.float32) / 10.0
        attitude_error = np.asarray((roll, pitch, yaw), dtype=np.float32) / np.pi
        rates = np.asarray(state.angular_velocity_body_rad_s, dtype=np.float32) / 10.0
        reward = (
            1.0
            - 6.0 * altitude_error**2
            - 1.0 * np.dot(velocity_error, velocity_error)
            - 2.0 * np.dot(attitude_error, attitude_error)
            - 0.5 * np.dot(rates, rates)
            - 0.02 * np.dot(action, action)
        )
        return float(reward - 25.0 if terminated else reward)

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
