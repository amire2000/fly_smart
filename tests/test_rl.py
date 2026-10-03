from dataclasses import replace

import numpy as np
import pytest

gym = pytest.importorskip("gymnasium")
from gymnasium.utils.env_checker import check_env

from fly_smart.rl import PyBulletHoverEnv
from fly_smart.rl.ppo_train import train_and_evaluate
from fly_smart.rl.pid_hover import HoverPidController
from fly_smart.adrc.adrc_hover import AdrcAxis, AdrcSettings, HoverAdrcController


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
        assert observation.shape == (10,)
        assert info["step"] == 0
        for _ in range(8):
            observation, reward, terminated, truncated, info = env.step(env.action_space.sample())
            assert np.isfinite(observation).all()
            assert np.isfinite(reward)
            if terminated or truncated:
                break
    finally:
        env.close()


def test_ppo_training_smoke_writes_evaluation(tmp_path):
    run_dir = train_and_evaluate(output_root=tmp_path, timesteps=8, episodes=1, max_episode_steps=2)
    assert (run_dir / "final_model.zip").exists()
    assert (run_dir / "evaluation.json").exists()


def test_pid_hover_action_is_bounded():
    env = PyBulletHoverEnv(max_episode_steps=1)
    try:
        state = env.simulator.reset()
        action = HoverPidController().action(state.drone)
        assert action.shape == (4,)
        assert np.all(action >= -1.0) and np.all(action <= 1.0)
    finally:
        env.close()


def test_hover_reward_penalizes_altitude_failure():
    env = PyBulletHoverEnv(max_episode_steps=1)
    try:
        state = env.simulator.reset().drone
        target_reward = env.task.reward(state, np.zeros(4, dtype=np.float32))
        failed_state = replace(state, position_m=(0.0, 0.0, 0.1))
        failed_reward = env.task.reward(failed_state, np.zeros(4, dtype=np.float32), terminated=True)
        assert target_reward > 0.0
        assert failed_reward < -20.0
    finally:
        env.close()


def test_adrc_output_is_bounded_and_observer_resets():
    env = PyBulletHoverEnv(max_episode_steps=1)
    try:
        state = env.simulator.reset().drone
        controller = HoverAdrcController()
        controller.reset(state)
        action = controller.action(state)
        assert np.all(action >= -1.0) and np.all(action <= 1.0)
    finally:
        env.close()
    axis = AdrcAxis(AdrcSettings(3.0, 1.0))
    axis.reset(2.0)
    assert axis.z1 == 2.0 and axis.z2 == 0.0 and axis.z3 == 0.0
