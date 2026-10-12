"""Interactive controls and live-telemetry adapter checks."""

import json
import socket

import pybullet as p
import pytest

from fly_smart.simulation.gui_helper import SimulationControls
from fly_smart.simulation.godot_bridge import GodotBridge
from fly_smart.simulation.plot_process import latest_sample
from fly_smart.simulation.runner import pacing_sleep_seconds, real_time_factor
from fly_smart.simulation.telemetry import FlightLog, append_telemetry_sample


def test_reset_pauses_and_is_consumed_once():
    controls = SimulationControls()
    controls.start()
    controls.request_reset()
    assert not controls.running
    assert controls.consume_reset()
    assert not controls.consume_reset()


def test_stop_is_consumed_once():
    controls = SimulationControls()
    controls.request_stop()
    assert controls.consume_stop()
    assert not controls.consume_stop()


def test_godot_pose_packet_carries_overlay_payload():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.settimeout(1.0)
    bridge = GodotBridge(port=receiver.getsockname()[1], event_port=0, control_port=0)
    try:
        bridge.publish_pose((1, 2, 3), (0, 0, 0, 1), (4, 5, 6), (0, 0, 0, 1), overlay={"bbox": [1, 2, 3, 4], "phase": "track"})
        payload, _ = receiver.recvfrom(4096)
        assert json.loads(payload)["overlay"] == {"bbox": [1, 2, 3, 4], "phase": "track"}
    finally:
        bridge.close()
        receiver.close()


@pytest.mark.parametrize("altitude", [20, 30, 50])
def test_godot_control_event_preserves_reset_selections(altitude):
    bridge = GodotBridge(event_port=0, control_port=0)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(
            json.dumps({"event": "simulation_control", "action": "reset", "target_distance_m": 150, "takeoff_altitude_m": altitude}).encode(),
            ("127.0.0.1", bridge._control_socket.getsockname()[1]),
        )
        assert bridge.read_control_event() == {"action": "reset", "target_distance_m": 150, "takeoff_altitude_m": altitude}
    finally:
        sender.close()
        bridge.close()


def test_godot_stop_control_event_is_valid():
    bridge = GodotBridge(event_port=0, control_port=0)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(
            json.dumps({"event": "simulation_control", "action": "stop"}).encode(),
            ("127.0.0.1", bridge._control_socket.getsockname()[1]),
        )
        assert bridge.read_control_event() == {"action": "stop"}
    finally:
        sender.close()
        bridge.close()


def test_real_time_factor_uses_simulated_over_wall_time():
    assert real_time_factor(10.0, 10.0) == 1.0
    assert real_time_factor(20.0, 10.0) == 2.0
    assert real_time_factor(5.0, 10.0) == 0.5
    assert real_time_factor(5.0, 0.0) == 0.0


def test_real_time_pacing_sleeps_only_for_remaining_deadline():
    assert pacing_sleep_seconds(1.0, 0.75) == 0.25
    assert pacing_sleep_seconds(1.0, 1.25) == 0.0


def test_plot_sample_round_trip_populates_child_log():
    source = FlightLog()
    source.time_s.append(1.0)
    source.phase.append("track")
    source.x_m.append(2.0)
    source.z_m.append(3.0)
    source.vx_mps.append(4.0)
    source.vz_mps.append(-1.0)
    for values in (
        source.command_vx_mps, source.command_vz_mps, source.pid_vz_target_mps,
        source.command_altitude_m, source.command_thrust_n, source.command_pitch_deg,
        source.measured_pitch_deg, source.ttc_s, source.raw_ttc_s, source.bbox_scale_px,
        source.bbox_growth_px_s, source.raw_bbox_growth_px_s,
        source.barometer_raw_altitude_m, source.barometer_filtered_altitude_m,
        source.frame_age_s, source.ttc_weight, source.bearing_alpha_deg, source.bearing_beta_deg,
        source.command_roll_deg, source.requested_vz_mps,
    ):
        values.append(1.0)
    source.visual_reason.append("ttc_qualified")
    source.guidance_reason.append("ttc_qualified")
    source.thrust_saturated.append(False)
    sample = latest_sample(source)
    assert sample is not None
    child = FlightLog()
    append_telemetry_sample(child, sample)
    assert child.time_s == [1.0]
    assert child.phase == ["track"]
    assert child.x_m == [2.0]
    assert child.bbox_scale_px == [1.0]


@pytest.mark.parametrize("altitude,expected", [(20, 20), (30, 30), (50, 50), (None, 15), (99, 15), ("bad", 15), (float("nan"), 15), (float("inf"), 15)])
def test_runner_reset_applies_altitude_and_preserves_previous_attempt(monkeypatch, tmp_path, altitude, expected):
    from fly_smart.simulation import runner
    from fly_smart.simulation.config import StrikeConfig

    class ResetBridge:
        """Deliver resets while recording the runner's authoritative settings."""

        def __init__(self):
            self.events = iter([{"action": "reset", "target_distance_m": 150, "takeoff_altitude_m": altitude},
                                {"action": "reset", "takeoff_altitude_m": 15}, {"action": "stop"}])
            self.applied = []

        def open(self):
            pass

        def clear_collision_events(self):
            pass

        def clear_control_events(self):
            pass

        def read_control_event(self):
            return next(self.events)

        def publish_pose(self, *args, **kwargs):
            self.applied.append(kwargs["target_control"])

        def close(self):
            pass

    heights = []
    guidance_type = runner.StrikeGuidance

    def record_guidance(config, max_thrust):
        heights.append(config.takeoff_altitude_m)
        return guidance_type(config, max_thrust)

    monkeypatch.setattr(runner, "StrikeGuidance", record_guidance)
    monkeypatch.setattr(runner, "start_plot_process", lambda *args: None)
    bridge = ResetBridge()
    client = p.connect(p.DIRECT)
    try:
        runner.StrikeSimulation(StrikeConfig(), godot=bridge).run(False, 1, None, None,
            summary=tmp_path / "summary.json", interactive=True)
    finally:
        p.disconnect(client)
    assert heights == [15, 15, expected, 15]
    assert bridge.applied == [{"distance_m": 30, "takeoff_altitude_m": 15},
                              {"distance_m": 150, "takeoff_altitude_m": expected},
                              {"distance_m": 150, "takeoff_altitude_m": 15}]
    previous = json.loads((tmp_path / "attempt-001/summary.json").read_text())
    assert previous["takeoff_altitude_m"] == 15
    assert previous["target"]["center_m"][0] == 24.75
    selected = json.loads((tmp_path / "attempt-002/summary.json").read_text())
    assert selected["takeoff_altitude_m"] == expected
