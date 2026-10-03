"""Command-line stationary hover check using the ADRC baseline."""

import argparse
from math import degrees, pi
from time import monotonic, sleep

import pybullet as p

from .adrc_hover import HoverAdrcController
from .bullet_hover_simulator import BulletHoverSimulator


def main(argv: list[str] | None = None) -> None:
    """Run the stationary 3 m ADRC hover acceptance check."""
    parser = argparse.ArgumentParser(description="Check 3 m hover with four ADRC actions")
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--bandwidth", type=float, default=6.0)
    args = parser.parse_args(argv)
    if args.seconds <= 0.0 or args.altitude <= 0.0 or args.bandwidth <= 0.0:
        parser.error("--seconds, --altitude, and --bandwidth must be positive")

    simulator = BulletHoverSimulator(hover_altitude_m=args.altitude, render_mode="human" if args.render else None)
    controller = HoverAdrcController(hover_altitude_m=args.altitude, bandwidth=args.bandwidth)
    state = simulator.reset()
    controller.reset(state.drone)
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

    passed = max_altitude_error <= 0.15 and max_attitude_error <= 5.0 * pi / 180.0
    print(f"max altitude error: {max_altitude_error:.3f} m")
    print(f"max attitude error: {degrees(max_attitude_error):.2f} deg")
    print("PASS" if passed else "FAIL")
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
