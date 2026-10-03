"""Command-line stationary hover check using the PID baseline."""

import argparse
from math import degrees
from time import monotonic, sleep

import pybullet as p

from .bullet_hover_simulator import BulletHoverSimulator
from .pid_hover import HoverPidController, PidGains


def _gains(values: list[float]) -> PidGains:
    """Convert one command-line gain triple to a typed gain set."""
    return PidGains(*values)


def main(argv: list[str] | None = None) -> None:
    """Run the stationary 3 m PID hover acceptance check."""
    parser = argparse.ArgumentParser(description="Check 3 m hover with four PID actions")
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--throttle-gains", nargs=3, type=float, default=[0.8, 0.25, 0.35], metavar=("KP", "KI", "KD"))
    parser.add_argument("--roll-gains", nargs=3, type=float, default=[1.4, 0.05, 0.25], metavar=("KP", "KI", "KD"))
    parser.add_argument("--pitch-gains", nargs=3, type=float, default=[1.4, 0.05, 0.25], metavar=("KP", "KI", "KD"))
    parser.add_argument("--yaw-gains", nargs=3, type=float, default=[0.8, 0.02, 0.15], metavar=("KP", "KI", "KD"))
    args = parser.parse_args(argv)
    if args.seconds <= 0.0 or args.altitude <= 0.0:
        parser.error("--seconds and --altitude must be positive")

    simulator = BulletHoverSimulator(hover_altitude_m=args.altitude, render_mode="human" if args.render else None)
    controller = HoverPidController(
        hover_altitude_m=args.altitude,
        throttle_gains=_gains(args.throttle_gains),
        roll_gains=_gains(args.roll_gains),
        pitch_gains=_gains(args.pitch_gains),
        yaw_gains=_gains(args.yaw_gains),
    )
    state = simulator.reset()
    controller.reset()
    max_altitude_error = 0.0
    max_attitude_error = 0.0
    next_tick = monotonic()
    try:
        for _ in range(round(args.seconds * simulator.settings.control_hz)):
            state = simulator.step(controller.action(state.drone))
            roll, pitch, yaw = p.getEulerFromQuaternion(state.drone.orientation_quaternion)
            max_altitude_error = max(max_altitude_error, abs(state.drone.position_m[2] - args.altitude))
            max_attitude_error = max(max_attitude_error, abs(roll), abs(pitch), abs(yaw))
            next_tick += 1.0 / simulator.settings.control_hz
            sleep(max(0.0, next_tick - monotonic()))
    finally:
        simulator.close()

    passed = max_altitude_error <= 0.15 and max_attitude_error <= 5.0 / 180.0 * 3.141592653589793
    print(f"max altitude error: {max_altitude_error:.3f} m")
    print(f"max attitude error: {degrees(max_attitude_error):.2f} deg")
    print("PASS" if passed else "FAIL")
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
