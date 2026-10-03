"""Command-line entry point for the Gymnasium hover environment."""

import argparse

from .hover_env import PyBulletHoverEnv


def main(argv: list[str] | None = None) -> None:
    """Run random hover episodes as an environment smoke test."""
    parser = argparse.ArgumentParser(description="Run the Fly Smart RL hover environment")
    parser.add_argument("--render", action="store_true", help="Show the PyBullet GUI")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--steps", type=int, default=1200, help="Maximum steps per episode")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    if args.episodes < 1 or args.steps < 1:
        parser.error("--episodes and --steps must be positive")

    env = PyBulletHoverEnv(render_mode="human" if args.render else None, max_episode_steps=args.steps)
    try:
        for episode in range(args.episodes):
            _, _ = env.reset(seed=args.seed + episode)
            for _ in range(args.steps):
                _, reward, terminated, truncated, info = env.step(env.action_space.sample())
                if terminated or truncated:
                    break
            print(f"episode {episode + 1}: steps={info['step']} reward={reward:.3f} reason={info['termination_reason']}")
    finally:
        env.close()


if __name__ == "__main__":
    main()
