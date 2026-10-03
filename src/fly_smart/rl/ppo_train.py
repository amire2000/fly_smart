"""PPO training and deterministic evaluation for the hover environment."""

import json
from datetime import datetime
from pathlib import Path

from .hover_env import PyBulletHoverEnv

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
except ModuleNotFoundError as exc:  # pragma: no cover - depends on optional extra
    raise ModuleNotFoundError("Install the RL extra with `uv sync --extra rl` to train PPO") from exc


def train_and_evaluate(
    *,
    output_root: Path = Path("outputs/rl_runs"),
    timesteps: int = 10_000,
    episodes: int = 5,
    max_episode_steps: int = 1_200,
    seed: int = 0,
    render: bool = False,
) -> Path:
    """Train PPO, save checkpoints, and write deterministic evaluation metrics."""
    if timesteps < 1 or episodes < 1 or max_episode_steps < 1:
        raise ValueError("timesteps, episodes, and max_episode_steps must be positive")
    run_dir = output_root / datetime.now().strftime("run-%Y%m%d-%H%M%S-%f")
    run_dir.mkdir(parents=True, exist_ok=False)

    train_env = PyBulletHoverEnv(max_episode_steps=max_episode_steps)
    try:
        checkpoint = CheckpointCallback(
            save_freq=max(1, timesteps // 5),
            save_path=str(run_dir / "checkpoints"),
            name_prefix="hover",
        )
        rollout_steps = max(8, min(128, max_episode_steps))
        model = PPO(
            "MlpPolicy",
            train_env,
            seed=seed,
            n_steps=rollout_steps,
            batch_size=rollout_steps,
            device="cpu",
            verbose=1,
        )
        model.learn(total_timesteps=timesteps, callback=checkpoint)
        model.save(str(run_dir / "final_model"))
    finally:
        train_env.close()

    eval_env = PyBulletHoverEnv(render_mode="human" if render else None, max_episode_steps=max_episode_steps)
    results: list[dict[str, object]] = []
    try:
        model = PPO.load(str(run_dir / "final_model"), env=eval_env, device="cpu")
        for episode in range(episodes):
            observation, _ = eval_env.reset(seed=seed + episode)
            total_reward = 0.0
            for _ in range(max_episode_steps):
                action, _ = model.predict(observation, deterministic=True)
                observation, reward, terminated, truncated, info = eval_env.step(action)
                total_reward += float(reward)
                if terminated or truncated:
                    break
            results.append(
                {
                    "episode": episode + 1,
                    "reward": total_reward,
                    "steps": info["step"],
                    "termination_reason": info["termination_reason"],
                    "altitude_m": info["altitude_m"],
                }
            )
    finally:
        eval_env.close()

    summary = {
        "timesteps": timesteps,
        "episodes": episodes,
        "max_episode_steps": max_episode_steps,
        "seed": seed,
        "results": results,
    }
    (run_dir / "evaluation.json").write_text(json.dumps(summary, indent=2) + "\n")
    return run_dir
