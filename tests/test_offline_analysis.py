import csv
from dataclasses import fields
from pathlib import Path

import pytest

from fly_smart.offline_analysis.loader import AnalysisLoadError, discover_attempts, load_attempt
from fly_smart.offline_analysis.playback import PlaybackController, TimelineMapper
from fly_smart.offline_analysis.state import FlightStatePresenter
from fly_smart.simulation.telemetry import FlightLog


SERIES_FIELDS = [
    field.name
    for field in fields(FlightLog)
    if field.name not in {"collision_time_s", "collision_position_m", "collision_velocity_mps"}
]


def write_attempt(folder: Path, *, with_video: bool = False) -> None:
    folder.mkdir(parents=True)
    with (folder / "telemetry.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=SERIES_FIELDS)
        writer.writeheader()
        for time_s, mode in ((0.0, "hold"), (1.0, "dy"), (2.0, "ttc")):
            writer.writerow({
                name: mode if name == "vertical_control_mode" else "track" if name == "phase" else str(time_s if name == "time_s" else 1.0)
                for name in SERIES_FIELDS
            })
    if with_video:
        (folder / "environment.mp4").write_bytes(b"not a real movie")


def test_loader_discovers_attempts_and_reads_full_telemetry(tmp_path):
    run = tmp_path / "run-1"
    write_attempt(run / "attempt-002")
    write_attempt(run / "attempt-001")

    assert [path.name for path in discover_attempts(run)] == ["attempt-001", "attempt-002"]
    attempt = load_attempt(run / "attempt-001")
    assert attempt.log.time_s == [0.0, 1.0, 2.0]
    assert attempt.log.vertical_control_mode == ["hold", "dy", "ttc"]
    assert attempt.video_path is None


def test_loader_accepts_a_direct_attempt_folder(tmp_path):
    attempt = tmp_path / "attempt-003"
    write_attempt(attempt)

    assert discover_attempts(attempt) == [attempt]


def test_loader_rejects_missing_required_column(tmp_path):
    attempt = tmp_path / "attempt-001"
    attempt.mkdir()
    (attempt / "telemetry.csv").write_text("time_s,phase\n0,track\n")

    with pytest.raises(AnalysisLoadError, match="missing required columns"):
        load_attempt(attempt)


def test_playback_steps_and_maps_recorded_time():
    playback = PlaybackController([0.0, 0.5, 1.5])
    assert playback.step(1).index == 1
    assert playback.step(1).index == 2
    assert playback.step(1).index == 2
    assert TimelineMapper.frame_index(1.5, 30.0, 60) == 45


def test_state_presenter_formats_nan_as_dash(tmp_path):
    attempt = tmp_path / "attempt-001"
    write_attempt(attempt)
    state = FlightStatePresenter().at(load_attempt(attempt).log, 2)

    assert state.vertical_mode == "TTC"
    assert "vertical control: TTC" in FlightStatePresenter().text(state)


def test_loader_rejects_empty_telemetry(tmp_path):
    attempt = tmp_path / "attempt-001"
    attempt.mkdir()
    (attempt / "telemetry.csv").write_text(",".join(SERIES_FIELDS) + "\n")

    with pytest.raises(AnalysisLoadError, match="no samples"):
        load_attempt(attempt)


def test_timeline_callback_is_ignored_during_programmatic_render():
    from fly_smart.offline_analysis.viewer import AnalyzerApplication

    class FakePlayback:
        def __init__(self):
            self.calls = 0
            self.state = type("State", (), {"playing": False})()

        def seek(self, index):
            self.calls += 1

    app = AnalyzerApplication.__new__(AnalyzerApplication)
    app._updating_timeline = True
    app.playback = FakePlayback()

    app._seek_from_slider("4")

    assert app.playback.calls == 0
