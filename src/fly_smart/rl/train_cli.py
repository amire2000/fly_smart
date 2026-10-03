"""Command-line entry point for PPO hover training."""

import argparse
from pathlib import Path

from .ppo_train import train_and_evaluate


def main(argv: list[str] | None = None) -> None:
    """Train and evaluate one PPO hover policy."""
    parser = argparse.ArgumentParser(description="Train and evaluate PPO on the Fly Smart hover task")
    parser.add_argument("--timesteps", type=int, default=10_000)
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--steps", type=int, default=1_200, help="Maximum steps per episode")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--render", action="store_true", help="Show the PyBullet GUI during evaluation")
    parser.add_argument("--output-root", type=Path, default=Path("outputs/rl_runs"))
    args = parser.parse_args(argv)
    if args.timesteps < 1 or args.episodes < 1 or args.steps < 1:
        parser.error("--timesteps, --episodes, and --steps must be positive")
    run_dir = train_and_evaluate(
        output_root=args.output_root,
        timesteps=args.timesteps,
        episodes=args.episodes,
        max_episode_steps=args.steps,
        seed=args.seed,
        render=args.render,
    )
    print(f"RL run folder: {run_dir}")


if __name__ == "__main__":
    main()
