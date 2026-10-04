"""Timestamp-driven playback primitives for offline flight review."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import time


@dataclass(frozen=True)
class PlaybackState:
    """Current playback cursor and transport state."""

    index: int
    playing: bool
    speed: float


class TimelineMapper:
    """Map recorded telemetry timestamps to video frame indices."""

    @staticmethod
    def frame_index(time_s: float, fps: float, frame_count: int | None = None) -> int:
        """Return the nearest clamped frame for a recorded timestamp."""
        if fps <= 0.0:
            return 0
        index = max(0, int(round(time_s * fps)))
        return min(index, frame_count - 1) if frame_count else index


class PlaybackController:
    """Advance a telemetry cursor using recorded timestamps and a speed scale."""

    def __init__(self, times: list[float]) -> None:
        if not times:
            raise ValueError("playback requires at least one timestamp")
        self._times = times
        self._index = 0
        self._playing = False
        self._speed = 1.0
        self._wall_started = 0.0
        self._flight_started = times[0]

    @property
    def state(self) -> PlaybackState:
        """Return an immutable snapshot for the UI."""
        return PlaybackState(self._index, self._playing, self._speed)

    def seek(self, index: int) -> PlaybackState:
        """Move to a valid telemetry sample index."""
        self._index = max(0, min(index, len(self._times) - 1))
        if self._playing:
            self._wall_started = time.monotonic()
            self._flight_started = self._times[self._index]
        return self.state

    def set_speed(self, speed: float) -> PlaybackState:
        """Set a positive playback multiplier."""
        if speed <= 0.0:
            raise ValueError("playback speed must be positive")
        if self._playing:
            self.tick()
            self._wall_started = time.monotonic()
            self._flight_started = self._times[self._index]
        self._speed = speed
        return self.state

    def step(self, delta: int) -> PlaybackState:
        """Move by a number of telemetry samples and pause playback."""
        self._playing = False
        return self.seek(self._index + delta)

    def play(self) -> PlaybackState:
        """Start timestamp-based playback unless already at the end."""
        if self._index >= len(self._times) - 1:
            self.seek(0)
        self._wall_started = time.monotonic()
        self._flight_started = self._times[self._index]
        self._playing = True
        return self.state

    def pause(self) -> PlaybackState:
        """Pause after applying the current wall-clock position."""
        self.tick()
        self._playing = False
        return self.state

    def tick(self, now: float | None = None) -> PlaybackState:
        """Advance to the sample at the current timestamp-scaled playhead."""
        if not self._playing:
            return self.state
        wall_now = time.monotonic() if now is None else now
        target_time = self._flight_started + (wall_now - self._wall_started) * self._speed
        self._index = min(bisect_right(self._times, target_time) - 1, len(self._times) - 1)
        if self._index >= len(self._times) - 1:
            self._index = len(self._times) - 1
            self._playing = False
        return self.state
