import json
import socket

from fly_smart.simulation.gui_helper import SimulationControls
from fly_smart.simulation.godot_bridge import GodotBridge


def test_reset_pauses_and_is_consumed_once():
    controls = SimulationControls()
    controls.start()
    controls.request_reset()
    assert not controls.running
    assert controls.consume_reset()
    assert not controls.consume_reset()


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
