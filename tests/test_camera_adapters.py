"""Camera projection and Godot capture metadata checks at the adapter seam."""

from math import radians
import socket

import numpy as np
import pybullet as p
import pytest

from fly_smart.mission import MissionConfig
from fly_smart.simulation.forward_camera import add_red_cube, forward_rgb
from fly_smart.simulation.godot_bridge import GodotBridge, HEADER
from fly_smart.trackers.red_target_detector import detect_red_box
from fly_smart.visual_guidance import VisualGuidanceEstimator, world_bearing


def free_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


@pytest.mark.parametrize("side", [1, 2, 3, 4, 5])
def test_small_red_patch_requires_resolved_extent_without_granting_ttc(side):
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb[260:260 + side, 320:320 + side] = (230, 0, 0)
    box = detect_red_box(rgb)[1]
    if side < 3:
        assert box is None
        return
    assert box == (320, 260, side, side)
    estimator = VisualGuidanceEstimator(MissionConfig())
    for frame in range(20):
        estimate = estimator.update(box, frame / 30, frame, (0, 0, 0), frame / 30)
        assert estimate.bearing_valid
        assert not estimate.ttc_reliable


@pytest.mark.parametrize("distance", [100, 150])
def test_pybullet_camera_detects_far_target_and_matches_compensated_bearing(distance):
    client = p.connect(p.DIRECT)
    try:
        drone = p.createMultiBody(baseMass=1, basePosition=(0, 0, 15),
            baseOrientation=p.getQuaternionFromEuler((radians(5), radians(5), radians(4))))
        add_red_cube((distance, 0, 1))
        image = forward_rgb(drone, p.ER_TINY_RENDERER, fov_deg=90)
        box = detect_red_box(image)[1]
        assert box is not None
        alpha, beta = world_bearing(box, MissionConfig(), (radians(5), radians(5), radians(4)), 0)
        assert alpha == pytest.approx(8 if distance == 100 else 5.33, abs=.5)
        assert beta == pytest.approx(0, abs=radians(.5))
        estimate = VisualGuidanceEstimator(MissionConfig()).update(box, 0, 0,
            (radians(5), radians(5), radians(4)), 0)
        assert estimate.bearing_valid
        assert not estimate.ttc_reliable
    finally:
        p.disconnect(client)


def test_godot_reader_requires_matching_pose_unique_frame_and_atomic_remap(tmp_path):
    path = tmp_path / "fpv.rgb"
    pixels = bytes(range(12))
    path.write_bytes(HEADER.pack(b"GFPV", 2, 2, 3, 0, 2, 12, 1) + pixels * 2)
    bridge = GodotBridge(path, event_port=free_udp_port(), control_port=free_udp_port())
    try:
        bridge.open()
        assert bridge.read_frame() is None  # Unknown pose never gets arrival-time freshness.
        bridge.publish_pose((0, 0, 0), (0, 0, 0, 1), (10, 0, 0), (0, 0, 0, 1),
                            captured_at_s=2., attitude_rad=(.1, .2, .3))
        assert np.array_equal(bridge.read_frame(), np.array(list(pixels)).reshape(2, 2, 3))
        assert bridge.frame_metadata == (2., 2, (.1, .2, .3))
        assert bridge.read_frame() is None
        bridge.publish_pose((0, 0, 0), (0, 0, 0, 1), (10, 0, 0), (0, 0, 0, 1),
                            captured_at_s=3., attitude_rad=(0, 0, 0))
        replacement = tmp_path / "next.rgb"
        replacement.write_bytes(HEADER.pack(b"GFPV", 1, 1, 3, 0, 4, 3, 2) + bytes(6))
        replacement.replace(path)
        assert bridge.read_frame().shape == (1, 1, 3)
    finally:
        bridge.close()


def test_contact_ends_mission_even_during_takeoff():
    from dataclasses import replace
    from fly_smart.simulation.config import StrikeConfig
    from fly_smart.simulation.runner import StrikeSimulation
    c = StrikeConfig()
    c = replace(c, simulation=replace(c.simulation, target_center=(c.launch_position[0], 0, 1), post_impact_seconds=0))
    client = p.connect(p.DIRECT)
    try:
        result = StrikeSimulation(c).run(False, .5, None, None)
        assert result.success
        assert result.simulated_time_s < .5
    finally:
        p.disconnect(client)
