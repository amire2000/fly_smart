"""Detect the red camera target with HSV thresholding and draw its bounding box."""

import argparse
from pathlib import Path
import time

import cv2
import numpy as np
import pybullet as p

from .common.drone_model import DEFAULT_DRONE_MODEL, DEFAULT_PHYSICS_SETTINGS
from .common.drone_physics import PhysicsEngine, clamp
from .common.flight_control import AttitudeController
from .common.pybullet_sensors import read_imu, read_state
from .common.pybullet_utils import create_world, draw_force_vectors
from .common.pid import PID
from .forward_camera import CAMERA_HZ, TARGET_ALTITUDE, add_environment_buildings, add_red_cube, forward_rgb

MODEL = DEFAULT_DRONE_MODEL
SETTINGS = DEFAULT_PHYSICS_SETTINGS
MASS = MODEL.mass_kg
PHYSICS_HZ = SETTINGS.physics_hz
TIME_STEP = SETTINGS.time_step_s
CONTROL_STEPS = SETTINGS.control_steps


def detect_red_box(rgb: np.ndarray) -> tuple[np.ndarray, tuple[int, int, int, int] | None]:
    """Return an annotated BGR frame and the largest red target bounding box."""
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    low_red = cv2.inRange(hsv, (0, 100, 80), (10, 255, 255))
    high_red = cv2.inRange(hsv, (170, 100, 80), (180, 255, 255))
    mask = cv2.morphologyEx(low_red | high_red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return bgr, None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 80:
        return bgr, None
    x, y, width, height = cv2.boundingRect(contour)
    cv2.rectangle(bgr, (x, y), (x + width, y + height), (0, 255, 255), 2)
    cv2.putText(bgr, f"red target: {width} x {height} px", (x, max(24, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    return bgr, (x, y, width, height)


def run(gui: bool, max_seconds: float) -> None:
    drone = create_world()
    engine = PhysicsEngine()
    add_red_cube()
    add_environment_buildings()
    altitude_pid = PID(kp=0.7, ki=0.05, kd=1.1, integral_limit=0.4)
    attitude_controller = AttitudeController()
    pwm = 1000.0
    torque = (0.0, 0.0, 0.0)
    force_lines = [-1, -1, -1, -1]
    renderer = p.ER_BULLET_HARDWARE_OPENGL if gui else p.ER_TINY_RENDERER

    for step in range(round(max_seconds / TIME_STEP)):
        state = read_state(drone)
        position = state.position_m
        vertical_velocity = state.linear_velocity_mps[2]
        if step % CONTROL_STEPS == 0:
            total_thrust = MASS * 9.81 + altitude_pid.update(TARGET_ALTITUDE - position[2], vertical_velocity)
            pwm = engine.pwm_from_thrust(clamp(total_thrust / 4, 0.0, MODEL.max_thrust_per_motor_n))
            torque = attitude_controller.update(read_imu(drone), yaw_target=0.0)
        flight_step = engine.step(drone, pwm, torque)

        if step % (PHYSICS_HZ // CAMERA_HZ) == 0:
            annotated, _ = detect_red_box(forward_rgb(drone, renderer))
            if gui:
                cv2.imshow("HSV red target detector", annotated)
                if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                    return

        if gui:
            draw_force_vectors(drone, flight_step, force_lines)
            time.sleep(TIME_STEP)


def self_check() -> None:
    drone = create_world()
    add_red_cube()
    _, box = detect_red_box(forward_rgb(drone, p.ER_TINY_RENDERER))
    assert box is not None, "HSV detector should find the red target cube"
    _, _, width, height = box
    assert width > 10 and height > 10, "Target bounding box should have visible size"
    print(f"Red target detector self-check passed: {width} x {height} px")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=30.0)
    args = parser.parse_args()
    client = p.connect(p.DIRECT if args.headless or args.self_check else p.GUI)
    try:
        if args.self_check:
            self_check()
        else:
            run(not args.headless, args.max_seconds)
    finally:
        cv2.destroyAllWindows()
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
