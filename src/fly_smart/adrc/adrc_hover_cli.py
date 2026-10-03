"""Command-line takeoff and hover check using the ADRC baseline."""

import argparse
from math import degrees, pi
from time import monotonic, sleep

import pybullet as p

from .adrc_hover import HoverAdrcController
from .bullet_hover_simulator import BulletHoverSimulator


def main(argv: list[str] | None = None) -> None:
    """Take off from near ground and run the 3 m ADRC hover check."""
    parser = argparse.ArgumentParser(description="Take off and hover at 3 m with four ADRC actions")
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--seconds", type=float, default=10.0, help="Hover hold time after takeoff")
    parser.add_argument("--takeoff-timeout", type=float, default=5.0)
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--initial-altitude", type=float, default=0.05)
    parser.add_argument("--bandwidth", type=float, default=3.0)
    args = parser.parse_args(argv)
    if args.seconds <= 0.0 or args.takeoff_timeout <= 0.0 or args.altitude <= 0.0 or args.initial_altitude < 0.05 or args.bandwidth <= 0.0:
        parser.error("--seconds, --takeoff-timeout, --altitude, and --bandwidth must be positive; --initial-altitude must be at least 0.05")

    simulator = BulletHoverSimulator(hover_altitude_m=args.altitude, render_mode="human" if args.render else None)
    controller = HoverAdrcController(hover_altitude_m=args.altitude, bandwidth=args.bandwidth)
    state = simulator.reset(args.initial_altitude)
    controller.reset(state.drone)
    takeoff_time_s: float | None = None
    hover_start_time_s: float | None = None
    max_hover_altitude_error = 0.0
    max_takeoff_altitude = args.initial_altitude
    max_attitude_error = 0.0
    hover_end_time_s: float | None = None
    debug_text_id = -1
    next_tick = monotonic()
    try:
        max_steps = round((args.takeoff_timeout + args.seconds) * simulator.settings.control_hz)
        for step in range(max_steps):
            action = controller.action(state.drone)
            state = simulator.step(action)
            sim_time_s = (step + 1) / simulator.settings.control_hz
            roll, pitch, yaw = p.getEulerFromQuaternion(state.drone.orientation_quaternion)
            max_attitude_error = max(max_attitude_error, abs(roll), abs(pitch), abs(yaw))
            altitude = state.drone.position_m[2]
            vertical_speed = abs(state.drone.linear_velocity_mps[2])
            if takeoff_time_s is None and altitude >= args.altitude - 0.15:
                takeoff_time_s = (step + 1) / simulator.settings.control_hz
            if hover_start_time_s is None:
                max_takeoff_altitude = max(max_takeoff_altitude, altitude)
                if takeoff_time_s is not None and abs(altitude - args.altitude) <= 0.15 and vertical_speed <= 0.5:
                    hover_start_time_s = (step + 1) / simulator.settings.control_hz
                    hover_end_time_s = hover_start_time_s + args.seconds
            else:
                max_hover_altitude_error = max(max_hover_altitude_error, abs(altitude - args.altitude))
            if args.render:
                phase = "TAKEOFF" if hover_start_time_s is None else "HOLD"
                text = (
                    f"ADRC {phase}  t={sim_time_s:.2f}s\n"
                    f"z={altitude:.2f}m  vz={state.drone.linear_velocity_mps[2]:.2f}m/s\n"
                    f"rpy=({degrees(roll):.1f}, {degrees(pitch):.1f}, {degrees(yaw):.1f}) deg\n"
                    f"action=({action[0]:.2f}, {action[1]:.2f}, {action[2]:.2f}, {action[3]:.2f})"
                )
                debug_text_id = p.addUserDebugText(text, (0.0, 0.0, args.altitude + 1.5), textColorRGB=(1.0, 0.8, 0.1), textSize=1.2, replaceItemUniqueId=debug_text_id)
            next_tick += 1.0 / simulator.settings.control_hz
            sleep(max(0.0, next_tick - monotonic()))
            if hover_end_time_s is not None and sim_time_s >= hover_end_time_s:
                break
    finally:
        simulator.close()

    final_altitude_error = abs(state.drone.position_m[2] - args.altitude)
    passed = hover_start_time_s is not None and max_hover_altitude_error <= 0.15 and final_altitude_error <= 0.15 and max_attitude_error <= 5.0 * pi / 180.0
    print(f"takeoff time: {takeoff_time_s:.2f} s" if takeoff_time_s is not None else "takeoff time: not reached")
    print(f"hover settle time: {hover_start_time_s:.2f} s" if hover_start_time_s is not None else "hover settle time: not reached")
    print(f"takeoff peak altitude: {max_takeoff_altitude:.3f} m")
    print(f"max hover altitude error: {max_hover_altitude_error:.3f} m")
    print(f"final altitude error: {final_altitude_error:.3f} m")
    print(f"max attitude error: {degrees(max_attitude_error):.2f} deg")
    print("PASS" if passed else "FAIL")
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
