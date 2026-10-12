"""Scenario configuration validation and compatibility checks."""

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


def test_default_scenario_uses_conservative_sensor_only_approach():
    config = load_yaml_config(Path("configs/scenario.yaml"))
    assert config.approach_pitch_deg == 7.0
    assert config.ttc_activation_s == 8.0
    assert config.max_vertical_position_correction_mps == 2.0


def test_sensor_settings_belong_to_simulation_and_keep_flat_access():
    config = load_yaml_config(Path("configs/scenario.yaml"))
    assert config.simulation.barometer_sample_hz == 40.0
    assert config.simulation.imu_sample_hz == 240.0
    assert not hasattr(config.runtime, "barometer_sample_hz")
    assert config.barometer_sample_hz == config.simulation.barometer_sample_hz


def test_runtime_sensor_section_requires_migration(tmp_path: Path):
    config_path = tmp_path / "scenario.yaml"
    config_path.write_text("runtime:\n  sensors:\n    imu:\n      sample_hz: 240\n")
    with pytest.raises(ValueError, match="move it to simulation.sensors"):
        load_yaml_config(config_path)


def test_descent_guard_settings_are_non_negative(tmp_path: Path):
    config_path = tmp_path / "scenario.yaml"
    config_path.write_text("runtime:\n  ttc:\n    ttc_activation_s: -1\n")
    with pytest.raises(ValueError, match="non-negative"):
        load_yaml_config(config_path)
    config_path.write_text("runtime:\n  vertical_control:\n    max_vertical_position_correction_mps: -1\n")
    with pytest.raises(ValueError, match="non-negative"):
        load_yaml_config(config_path)


@pytest.mark.parametrize("setting,value", [("ttc_blend_s", 0), ("ttc_min_samples", 1),
                                           ("vertical_slew_mps2", -1), ("frame_max_age_s", ".nan")])
def test_new_guidance_settings_reject_invalid_values(tmp_path, setting, value):
    path = tmp_path / "invalid.yaml"
    path.write_text(f"runtime:\n  guidance:\n    {setting}: {value}\n")
    with pytest.raises(ValueError):
        load_yaml_config(path)


def test_all_shipped_configs_load_with_core_runtime_defaults():
    for path in Path("configs").glob("*.yaml"):
        config = load_yaml_config(path)
        assert config.camera_width_px > 0
        assert config.vertical_velocity_pid_gains[1] > 0
        assert config.roll_attitude_pid_gains == (.02, 0, .01)
