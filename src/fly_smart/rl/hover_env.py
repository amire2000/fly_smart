"""Gymnasium environment for learning state-based hover stabilization."""

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ModuleNotFoundError as exc:  # pragma: no cover - exercised without the optional extra
    raise ModuleNotFoundError("Install the RL extra with `uv sync --extra rl` to use fly_smart.rl") from exc

from .bullet_hover_simulator import BulletHoverSimulator
from .hover_task import HoverTask
from .pybullet_progress import PyBulletProgressRenderer


class PyBulletHoverEnv(gym.Env):
    """Train a policy to stabilize the existing physical engine at hover."""

    metadata = {"render_modes": ["human"], "render_fps": 120}

    def __init__(self, render_mode: str | None = None, hover_altitude_m: float = 3.0, max_episode_steps: int = 1200) -> None:
        super().__init__()
        self.render_mode = render_mode
        self.max_episode_steps = max_episode_steps
        self.simulator = BulletHoverSimulator(hover_altitude_m=hover_altitude_m, render_mode=render_mode)
        self.task = HoverTask(hover_altitude_m)
        self.renderer = PyBulletProgressRenderer(hover_altitude_m) if render_mode == "human" else None
        self.action_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(15,), dtype=np.float32)
        self._step_count = 0
        self._last_action = np.zeros(4, dtype=np.float32)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        """Reset the world and return the initial normalized observation."""
        del options
        super().reset(seed=seed)
        state = self.simulator.reset(seed)
        self._step_count = 0
        self._last_action.fill(0.0)
        return self.task.observation(state.drone, state.physics), {"step": 0}

    def step(self, action: np.ndarray):
        """Advance one control interval and return the Gymnasium transition."""
        action = np.asarray(action, dtype=np.float32)
        if action.shape != self.action_space.shape:
            raise ValueError(f"action must have shape {self.action_space.shape}")
        action = np.clip(action, -1.0, 1.0)
        state = self.simulator.step(action)
        self._step_count += 1
        observation = self.task.observation(state.drone, state.physics)
        reward = self.task.reward(state.drone, action) - 0.01 * float(np.dot(action - self._last_action, action - self._last_action))
        terminated, reason = self.task.terminated(state.drone)
        truncated = self._step_count >= self.max_episode_steps and not terminated
        self._last_action = action.copy()
        info = {"step": self._step_count, "termination_reason": reason, "altitude_m": state.drone.position_m[2]}
        if self.renderer:
            self.renderer.update(state.drone, self._step_count, reward, terminated, truncated)
        return observation, reward, terminated, truncated, info

    def render(self):
        """Keep the PyBullet GUI responsive; state is updated during ``step``."""
        return None

    def close(self) -> None:
        """Close the renderer and PyBullet client."""
        self.simulator.close()
