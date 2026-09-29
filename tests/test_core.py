import numpy as np

from fly_smart.guidance import GuidanceInput, StrikeGuidance
from fly_smart.mission import MissionConfig
from fly_smart.red_target_detector import detect_red_box
from fly_smart.sensing import BarometerReading
from fly_smart.ttc import BboxTtcTracker


def test_core_guidance_and_vision_do_not_need_simulation():
    config = MissionConfig()
    tracker = BboxTtcTracker(config)
    assert tracker.update((0, 0, 20, 20), 0.0) is None
    observation = tracker.update((0, 0, 30, 30), 0.1)
    assert observation is not None and observation.ttc_s > 0
    guidance = StrikeGuidance(config)
    command = guidance.update(GuidanceInput(0.0, BarometerReading(0.0, 0.0), None, None, False, False))
    assert command.thrust_n > config.hover_thrust_n

    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    frame[20:70, 20:70] = (255, 0, 0)
    _, box = detect_red_box(frame)
    assert box == (20, 20, 50, 50)
