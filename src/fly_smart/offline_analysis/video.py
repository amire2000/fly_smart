"""Video sources used by the offline analyzer."""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

import cv2
import numpy as np


class VideoSource(Protocol):
    """Small interface required by playback and the viewer."""

    @property
    def available(self) -> bool: ...

    @property
    def fps(self) -> float: ...

    @property
    def frame_count(self) -> int | None: ...

    def frame_at(self, index: int) -> np.ndarray | None: ...

    def close(self) -> None: ...


class OpenCvVideoSource:
    """Random-access video reader backed by OpenCV."""

    def __init__(self, path: Path) -> None:
        self._capture = cv2.VideoCapture(str(path))
        if not self._capture.isOpened():
            self._capture.release()
            raise OSError(f"Could not open video: {path}")
        self._fps = float(self._capture.get(cv2.CAP_PROP_FPS))
        self._frame_count = int(self._capture.get(cv2.CAP_PROP_FRAME_COUNT)) or None

    @property
    def available(self) -> bool:
        return True

    @property
    def fps(self) -> float:
        return self._fps

    @property
    def frame_count(self) -> int | None:
        return self._frame_count

    def frame_at(self, index: int) -> np.ndarray | None:
        """Seek and return one RGB frame, or None when decoding fails."""
        self._capture.set(cv2.CAP_PROP_POS_FRAMES, max(0, index))
        ok, frame = self._capture.read()
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) if ok else None

    def close(self) -> None:
        self._capture.release()


class NoVideoSource:
    """Null object that keeps telemetry review usable without a movie."""

    @property
    def available(self) -> bool:
        return False

    @property
    def fps(self) -> float:
        return 0.0

    @property
    def frame_count(self) -> int | None:
        return None

    def frame_at(self, index: int) -> np.ndarray | None:
        return None

    def close(self) -> None:
        return None
