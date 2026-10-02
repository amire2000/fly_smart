import pickle

from fly_smart.simulation.config import StrikeConfig


def test_strike_config_can_cross_spawn_pickle_boundary():
    restored = pickle.loads(pickle.dumps(StrikeConfig()))
    assert restored.runtime.camera_hz == StrikeConfig().runtime.camera_hz
