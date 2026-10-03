"""Standalone PyBullet adapter used by the ADRC hover example."""

from dataclasses import dataclass

import numpy as np
import pybullet as p

from ..simulation.drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS, DroneModel, DroneState, PhysicsSettings, PhysicsStep
from ..simulation.drone_physics import PWM_MAX, PWM_MIN, PhysicsEngine
from ..simulation.pybullet_utils import create_world, reset_drone


@dataclass(frozen=True)
class HoverSimulatorState:
    """State returned by one standalone simulator step."""

    drone: DroneState
    physics: PhysicsStep | None


class BulletHoverSimulator:
    """Own one resettable PyBullet world without Gymnasium dependencies."""

    def __init__(
        self,
        model: DroneModel = DEFAULT_DRONE_MODEL,
        settings: PhysicsSettings = DEFAULT_PHYSICS_SETTINGS,
        hover_altitude_m: float = 3.0,
        torque_limits_nm: tuple[float, float, float] = (0.05, 0.05, 0.02),
        render_mode: str | None = None,
    ) -> None:
        if render_mode not in (None, "human"):
            raise ValueError("render_mode must be None or 'human'")
        self.model = model
        self.settings = settings
        self.hover_altitude_m = hover_altitude_m
        self.torque_limits_nm = torque_limits_nm
        self.render_mode = render_mode
        self.drone: int | None = None
        self.engine = PhysicsEngine(model, settings)
        self._client_id: int | None = None

    def reset(self, initial_altitude_m: float = 0.05) -> HoverSimulatorState:
        """Create a fresh world and place the drone at the requested start altitude."""
        if initial_altitude_m < 0.05:
            raise ValueError("initial_altitude_m must be at least 0.05 m")
        if self._client_id is None:
            self._client_id = p.connect(p.GUI if self.render_mode == "human" else p.DIRECT)
        self.drone = create_world(self.model, self.settings)
        reset_drone(self.drone, (0.0, 0.0, initial_altitude_m))
        self.engine.reset()
        if self.render_mode == "human":
            p.resetDebugVisualizerCamera(8.0, 35.0, -25.0, (0.0, 0.0, self.hover_altitude_m))
        return HoverSimulatorState(self._state(), None)

    def step(self, action: np.ndarray) -> HoverSimulatorState:
        """Apply one normalized throttle/torque action for one control interval."""
        if self.drone is None:
            raise RuntimeError("reset() must be called before step()")
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (4,):
            raise ValueError("action must have shape (4,)")
        collective_pwm = float(np.interp(action[0], (-1.0, 1.0), (PWM_MIN, PWM_MAX)))
        torque = tuple(float(value * limit) for value, limit in zip(action[1:], self.torque_limits_nm))
        physics_step = None
        for _ in range(self.settings.control_steps):
            physics_step = self.engine.step(self.drone, collective_pwm, torque)
        return HoverSimulatorState(self._state(), physics_step)

    def close(self) -> None:
        """Disconnect the standalone PyBullet world."""
        if self._client_id is not None and p.isConnected(self._client_id):
            p.disconnect(self._client_id)
        self._client_id = None
        self.drone = None

    def _state(self) -> DroneState:
        """Read the current rigid-body state from PyBullet."""
        if self.drone is None:
            raise RuntimeError("reset() must be called before reading state")
        position, orientation = p.getBasePositionAndOrientation(self.drone)
        velocity, angular_velocity = p.getBaseVelocity(self.drone)
        rotation = p.getMatrixFromQuaternion(orientation)
        body_rates = (
            rotation[0] * angular_velocity[0] + rotation[3] * angular_velocity[1] + rotation[6] * angular_velocity[2],
            rotation[1] * angular_velocity[0] + rotation[4] * angular_velocity[1] + rotation[7] * angular_velocity[2],
            rotation[2] * angular_velocity[0] + rotation[5] * angular_velocity[1] + rotation[8] * angular_velocity[2],
        )
        return DroneState(tuple(position), tuple(velocity), tuple(orientation), body_rates)
