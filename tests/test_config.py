from pathlib import Path

import pytest

from fly_smart.simulation.config_loader import load_yaml_config


def test_ttc_unavailable_descent_velocity_yaml_validation(tmp_path: Path):
    config_path = tmp_path / "scenario.yaml"
    config_path.write_text("runtime:\n  ttc:\n    ttc_unavailable_descent_velocity_mps: 0\n")
    assert load_yaml_config(config_path).ttc_unavailable_descent_velocity_mps == 0.0

    config_path.write_text("runtime:\n  ttc:\n    ttc_unavailable_descent_velocity_mps: -1\n")
    with pytest.raises(ValueError, match="non-negative"):
        load_yaml_config(config_path)


def test_ttc_unavailable_pitch_boost_yaml_validation(tmp_path: Path):
    config_path = tmp_path / "scenario.yaml"
    config_path.write_text("runtime:\n  ttc:\n    ttc_unavailable_pitch_boost_deg: 5\n")
    assert load_yaml_config(config_path).ttc_unavailable_pitch_boost_deg == 5.0

    config_path.write_text("runtime:\n  ttc:\n    ttc_unavailable_pitch_boost_deg: -1\n")
    with pytest.raises(ValueError, match="non-negative"):
        load_yaml_config(config_path)


def test_default_scenario_enables_pitch_boost_for_invalid_ttc():
    config = load_yaml_config(Path("configs/scenario.yaml"))
    assert config.ttc_unavailable_pitch_boost_deg == 5.0
    assert config.ttc_activation_s == 8.0
    assert config.max_vertical_position_correction_mps == 2.0


def test_descent_guard_settings_are_non_negative(tmp_path: Path):
    config_path = tmp_path / "scenario.yaml"
    config_path.write_text("runtime:\n  ttc:\n    ttc_activation_s: -1\n")
    with pytest.raises(ValueError, match="non-negative"):
        load_yaml_config(config_path)
    config_path.write_text("runtime:\n  vertical_control:\n    max_vertical_position_correction_mps: -1\n")
    with pytest.raises(ValueError, match="non-negative"):
        load_yaml_config(config_path)
