import numpy as np
import pytest

gym = pytest.importorskip("gymnasium")
from gymnasium.utils.env_checker import check_env

from fly_smart.rl import PyBulletHoverEnv


def test_hover_environment_passes_gymnasium_checker():
    env = PyBulletHoverEnv(max_episode_steps=4)
    try:
        check_env(env, skip_render_check=True)
    finally:
        env.close()


def test_hover_environment_random_rollout_is_finite():
    env = PyBulletHoverEnv(max_episode_steps=8)
    try:
        observation, info = env.reset(seed=3)
        assert observation.shape == (15,)
        assert info["step"] == 0
        for _ in range(8):
            observation, reward, terminated, truncated, info = env.step(env.action_space.sample())
            assert np.isfinite(observation).all()
            assert np.isfinite(reward)
            if terminated or truncated:
                break
    finally:
        env.close()
