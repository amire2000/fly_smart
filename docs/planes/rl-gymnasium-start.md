# Starting reinforcement learning with the PyBullet engine

The first RL environment is a state-based hover task. Install the optional
dependency and run a checker/random rollout:

```bash
uv sync --extra dev --extra rl
uv run pytest tests/test_rl.py -q
uv run fly-smart-rl --render
uv run fly-smart-rl-train --timesteps 10000 --episodes 5
```

```python
from fly_smart.rl import PyBulletHoverEnv

env = PyBulletHoverEnv(render_mode="human")
observation, info = env.reset(seed=0)
for _ in range(1200):
    observation, reward, terminated, truncated, info = env.step(env.action_space.sample())
    if terminated or truncated:
        break
env.close()
```

Use `render_mode=None` for headless training. The agent controls normalized
collective PWM and roll/pitch/yaw torque. The PyBullet GUI is an observer of
the same physical engine; it does not affect reward or episode state.

The hover observation has 10 values: altitude error, 3D velocity,
roll/pitch/yaw, and body angular rates. PPO checkpoints trained with the
previous 15-value observation are incompatible and must be retrained.

`fly-smart-rl` runs one headless random episode by default. Use
`--episodes N`, `--steps N`, and `--render` to inspect more episodes in the
PyBullet GUI.

`fly-smart-rl-train` trains PPO, saves checkpoints and `final_model.zip`, then
writes deterministic evaluation results to `outputs/rl_runs/<run>/evaluation.json`.
