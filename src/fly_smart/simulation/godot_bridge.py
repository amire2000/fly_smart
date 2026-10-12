"""Small bridge for the Godot renderer used by the ``godot-render`` branch."""

from __future__ import annotations

from collections import OrderedDict
import json
import mmap
import os
from pathlib import Path
import socket
import struct

import numpy as np


HEADER = struct.Struct("<4s7I")
HEADER_BYTES = HEADER.size
COLLISION_PORT = 9101
CONTROL_PORT = 9102


class GodotBridge:
    """Publish PyBullet poses and read the newest Godot RGB camera frame."""

    def __init__(self, path: Path = Path("/dev/shm/fly_smart_fpv.rgb"), port: int = 9100, event_port: int = COLLISION_PORT, control_port: int = CONTROL_PORT) -> None:
        self.path = path
        self.destination = ("127.0.0.1", port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._event_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._event_socket.bind(("127.0.0.1", event_port))
        self._event_socket.setblocking(False)
        self._control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._control_socket.bind(("127.0.0.1", control_port))
        self._control_socket.setblocking(False)
        self._file = None
        self._mapping = None
        self._pose_id = 0
        self._poses = OrderedDict()
        self._last_sequence = None
        self._last_pose_id = None
        self.frame_metadata = None

    def open(self) -> None:
        """Open the shared-memory-backed file created by Godot."""
        if not self.path.is_file():
            raise RuntimeError(f"Godot shared-memory file is missing: {self.path}")
        self._file = self.path.open("rb")
        self._mapping = mmap.mmap(self._file.fileno(), 0, access=mmap.ACCESS_READ)

    def publish_pose(self, drone_position, drone_orientation, target_position, target_orientation, reset: bool = False, overlay: dict | None = None, target_control: dict | None = None, captured_at_s: float | None = None, attitude_rad: tuple[float, float, float] | None = None, camera: dict | None = None) -> None:
        """Send the latest PyBullet poses; Godot drains old packets each frame."""
        payload = {
            "drone": {"p": list(drone_position), "q": list(drone_orientation)},
            "target": {"p": list(target_position), "q": list(target_orientation)},
        }
        if captured_at_s is not None and attitude_rad is not None:
            self._pose_id = (self._pose_id + 1) % (2**32) or 1
            payload["pose_id"] = self._pose_id
            self._poses[self._pose_id] = (captured_at_s, attitude_rad)
            while len(self._poses) > 256:
                self._poses.popitem(last=False)
        if camera is not None:
            payload["camera"] = camera
        if reset:
            self._poses.clear()
            self._last_sequence = None
            self._last_pose_id = None
            self.frame_metadata = None
            payload["reset"] = True
        if overlay is not None:
            payload["overlay"] = overlay
        if target_control is not None:
            payload["target_control"] = target_control
        self._socket.sendto(json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8"), self.destination)

    def read_frame(self) -> np.ndarray | None:
        """Return one RGB frame, or ``None`` while Godot is writing a frame."""
        if self._mapping is None:
            return None
        if self._file and self.path.stat().st_ino != os.fstat(self._file.fileno()).st_ino:
            self._mapping.close()
            self._file.close()
            self.open()
            self._last_sequence = None
        if len(self._mapping) < HEADER_BYTES:
            return None
        header = HEADER.unpack_from(self._mapping)
        magic, width, height, channels, slot, sequence, frame_bytes, pose_id = header
        if magic != b"GFPV" or channels != 3 or slot > 1 or sequence == 0 or sequence & 1:
            return None
        if frame_bytes != width * height * channels or len(self._mapping) < HEADER_BYTES + 2 * frame_bytes:
            return None
        if sequence == self._last_sequence or pose_id == self._last_pose_id or pose_id not in self._poses:
            return None
        start = HEADER_BYTES + slot * frame_bytes
        data = self._mapping[start : start + frame_bytes]
        if HEADER.unpack_from(self._mapping) != header:
            return None
        self._last_sequence = sequence
        self._last_pose_id = pose_id
        time_s, attitude = self._poses[pose_id]
        self.frame_metadata = (time_s, sequence, attitude)
        return np.frombuffer(data, dtype=np.uint8).reshape(height, width, channels).copy()

    def read_collision_event(self) -> str | None:
        """Return Godot's newest target or obstacle collision event, if any."""
        event = None
        while True:
            try:
                payload, _ = self._event_socket.recvfrom(1024)
            except BlockingIOError:
                return event
            try:
                value = json.loads(payload)
            except (TypeError, json.JSONDecodeError):
                continue
            kind = value.get("kind") if isinstance(value, dict) else None
            if isinstance(value, dict) and value.get("event") == "collision" and kind in {"target", "obstacle"}:
                event = kind

    def clear_collision_events(self) -> None:
        """Discard stale Godot collision events before a restarted attempt."""
        while True:
            try:
                self._event_socket.recvfrom(1024)
            except BlockingIOError:
                return

    def read_control_event(self) -> dict | None:
        """Return the newest valid Godot command, including its selection."""
        event = None
        while True:
            try:
                payload, _ = self._control_socket.recvfrom(1024)
            except BlockingIOError:
                return event
            try:
                value = json.loads(payload)
            except (TypeError, json.JSONDecodeError):
                continue
            if isinstance(value, dict) and value.get("event") == "simulation_control" and value.get("action") in {"start", "reset", "stop"}:
                event = {"action": value["action"]}
                if "target_distance_m" in value:
                    event["target_distance_m"] = value["target_distance_m"]
                if "takeoff_altitude_m" in value:
                    event["takeoff_altitude_m"] = value["takeoff_altitude_m"]

    def clear_control_events(self) -> None:
        """Discard stale UI commands before an interactive attempt starts."""
        while True:
            try:
                self._control_socket.recvfrom(1024)
            except BlockingIOError:
                return

    def close(self) -> None:
        if self._mapping is not None:
            self._mapping.close()
            self._mapping = None
        if self._file is not None:
            self._file.close()
            self._file = None
        self._socket.close()
        self._event_socket.close()
        self._control_socket.close()

    def __enter__(self) -> "GodotBridge":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
