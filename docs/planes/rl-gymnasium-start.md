# Starting reinforcement learning with the PyBullet engine

The first RL environment is a state-based hover task. Install the optional
dependency and run a checker/random rollout:

```bash
uv sync --extra dev --extra rl
uv run pytest tests/test_rl.py -q
uv run fly-smart-rl --render
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

`fly-smart-rl` runs one headless random episode by default. Use
`--episodes N`, `--steps N`, and `--render` to inspect more episodes in the
PyBullet GUI.
