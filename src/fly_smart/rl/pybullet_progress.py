"""Optional PyBullet GUI progress observer for RL episodes."""

import pybullet as p

from ..simulation.drone_model import DroneState


class PyBulletProgressRenderer:
    """Show lightweight episode telemetry without owning simulation state."""

    def __init__(self, hover_altitude_m: float) -> None:
        self.hover_altitude_m = hover_altitude_m
        self._text_id = -1

    def update(self, state: DroneState, step: int, reward: float, terminated: bool, truncated: bool) -> None:
        """Update the debug camera and textual episode overlay."""
        roll, pitch, yaw = p.getEulerFromQuaternion(state.orientation_quaternion)
        p.resetDebugVisualizerCamera(8.0, 35.0, -25.0, (0.0, 0.0, self.hover_altitude_m))
        text = f"RL hover step {step}  reward {reward:.3f}\nz {state.position_m[2]:.2f}  v {state.linear_velocity_mps}\nrpy {(roll, pitch, yaw)}"
        if terminated or truncated:
            text += "\nfinished"
        self._text_id = p.addUserDebugText(text, (0.0, 0.0, self.hover_altitude_m + 2.0), textColorRGB=(1, 1, 1), textSize=1.2, replaceItemUniqueId=self._text_id)
