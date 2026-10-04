"""Load recorded simulation attempts for offline inspection."""

from __future__ import annotations

import csv
from dataclasses import dataclass, fields
import json
from pathlib import Path
from typing import Any

from ..simulation.telemetry import FlightLog


class AnalysisLoadError(RuntimeError):
    """Raised when an attempt cannot provide valid telemetry for analysis."""


@dataclass(frozen=True)
class OfflineAttempt:
    """Loaded telemetry and metadata for one recorded attempt."""

    folder: Path
    log: FlightLog
    summary: dict[str, Any]
    settings: dict[str, Any]
    video_path: Path | None


_NON_SERIES_FIELDS = {"collision_time_s", "collision_position_m", "collision_velocity_mps"}
_STRING_FIELDS = {"phase", "vertical_control_mode"}


def discover_attempts(run_folder: Path) -> list[Path]:
    """Return attempt directories, accepting a run or direct attempt folder."""
    if not run_folder.is_dir():
        raise AnalysisLoadError(f"Run folder does not exist: {run_folder}")
    if (run_folder / "telemetry.csv").is_file():
        return [run_folder]
    attempts = sorted(path for path in run_folder.glob("attempt-*") if path.is_dir())
    if not attempts:
        raise AnalysisLoadError(f"No attempt-* folders found in: {run_folder}")
    return attempts


def load_attempt(attempt_folder: Path) -> OfflineAttempt:
    """Load one attempt, failing clearly when telemetry is absent or invalid."""
    telemetry_path = attempt_folder / "telemetry.csv"
    if not telemetry_path.is_file():
        raise AnalysisLoadError(f"Missing telemetry.csv: {telemetry_path}")
    try:
        log = _load_csv(telemetry_path)
        summary = _load_json_if_present(attempt_folder / "summary.json")
        settings = _load_json_if_present(attempt_folder / "settings.json")
    except (OSError, csv.Error, ValueError, TypeError, json.JSONDecodeError) as exc:
        raise AnalysisLoadError(f"Invalid analysis data in {attempt_folder}: {exc}") from exc
    video = attempt_folder / "environment.mp4"
    return OfflineAttempt(attempt_folder, log, summary, settings, video if video.is_file() else None)


def _load_json_if_present(path: Path) -> dict[str, Any]:
    """Read an optional JSON metadata file as an object."""
    if not path.is_file():
        return {}
    with path.open() as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object in {path}")
    return value


def _load_csv(path: Path) -> FlightLog:
    """Load the persisted FlightLog series without changing its CSV schema."""
    series_fields = [field.name for field in fields(FlightLog) if field.name not in _NON_SERIES_FIELDS]
    required = {"time_s", "phase", "vertical_control_mode"}
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        headers = set(reader.fieldnames or ())
        missing = required - headers
        if missing:
            raise ValueError(f"missing required columns: {', '.join(sorted(missing))}")
        log = FlightLog()
        for row_number, row in enumerate(reader, start=2):
            try:
                for name in series_fields:
                    value = row.get(name)
                    if value is None or value == "":
                        raise ValueError(f"empty value for {name}")
                    if name in _STRING_FIELDS:
                        parsed: object = value
                    else:
                        parsed = float(value)
                    getattr(log, name).append(parsed)
            except (KeyError, ValueError, TypeError) as exc:
                raise ValueError(f"row {row_number}: {exc}") from exc
    if not log.time_s:
        raise ValueError("telemetry.csv contains no samples")
    if any(later < earlier for earlier, later in zip(log.time_s, log.time_s[1:])):
        raise ValueError("time_s must be non-decreasing")
    return log
