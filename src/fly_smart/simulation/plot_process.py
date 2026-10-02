"""Best-effort UDP live telemetry publisher and Matplotlib child process."""

from __future__ import annotations

import json
import multiprocessing as mp
from pathlib import Path
import socket
import time

from .config import SceneConfig, StrikeConfig
from .telemetry import FlightLog, append_telemetry_sample, make_plot, move_plot_window, refresh_plot


PLOT_PORT = 9103
PLOT_HZ = 5.0


def start_plot_process(config: StrikeConfig, scene: SceneConfig, scenario_name: str, output: Path | None) -> tuple[mp.Process, socket.socket] | None:
    """Start the display child and return its non-blocking UDP sender."""
    if output is None:
        return None
    context = mp.get_context("spawn")
    process = context.Process(target=run_plot_process, args=(config, scene, scenario_name), daemon=True)
    process.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.setblocking(False)
    return process, sender


def send_plot_message(sender: socket.socket, message: dict[str, object]) -> None:
    """Send one display message without allowing plotter problems to stop physics."""
    try:
        sender.sendto(json.dumps(message, separators=(",", ":")).encode("utf-8"), ("127.0.0.1", PLOT_PORT))
    except OSError:
        pass


def run_plot_process(config: StrikeConfig, scene: SceneConfig, scenario_name: str) -> None:
    """Receive telemetry and own the live Matplotlib figure in a child process."""
    import matplotlib.pyplot as plt

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        receiver.bind(("127.0.0.1", PLOT_PORT))
        receiver.setblocking(False)
        plot = make_plot(config, scene, scenario_name)
        plt.ion()
        plot.figure.canvas.manager.set_window_title(f"TTC strike telemetry — {scene.drone_profile} / {scenario_name}")
        plt.show(block=False)
        if hasattr(plot.figure.canvas.manager, "window"):
            move_plot_window(plot, config.plot_window_position_px)
        log = FlightLog()
        next_refresh = time.monotonic()
        running = True
        while running:
            while True:
                try:
                    payload, _ = receiver.recvfrom(8192)
                except BlockingIOError:
                    break
                try:
                    message = json.loads(payload)
                except (TypeError, json.JSONDecodeError):
                    continue
                if not isinstance(message, dict):
                    continue
                kind = message.get("type")
                if kind == "telemetry_sample":
                    append_telemetry_sample(log, message)
                elif kind == "target":
                    center = message.get("center_m")
                    if isinstance(center, list) and len(center) == 3:
                        plot.target_marker.set_offsets([[float(center[0]), float(center[2])]])
                elif kind == "reset":
                    log = FlightLog()
                elif kind == "collision":
                    log.collision_time_s = float(message["time_s"])
                elif kind == "close":
                    running = False
                    break
            now = time.monotonic()
            if now >= next_refresh:
                refresh_plot(plot, log)
                next_refresh = now + 1.0 / PLOT_HZ
            # Keep Matplotlib responsive without spinning a CPU core while
            # waiting for the next telemetry refresh.
            plt.pause(0.02)
        plt.close(plot.figure)
    except OSError:
        return
    finally:
        receiver.close()


def latest_sample(log: FlightLog) -> dict[str, object] | None:
    """Serialize the newest logged sample for the display process."""
    if not log.time_s:
        return None
    index = -1
    return {
        "type": "telemetry_sample",
        "t": log.time_s[index],
        "phase": log.phase[index],
        "x": log.x_m[index],
        "z": log.z_m[index],
        "vx": log.vx_mps[index],
        "vz": log.vz_mps[index],
        "command_vx": log.command_vx_mps[index],
        "command_vz": log.command_vz_mps[index],
        "pid_vz": log.pid_vz_target_mps[index],
        "command_altitude": log.command_altitude_m[index],
        "thrust": log.command_thrust_n[index],
        "pitch": log.command_pitch_deg[index],
        "measured_pitch": log.measured_pitch_deg[index],
        "ttc": log.ttc_s[index],
        "raw_ttc": log.raw_ttc_s[index],
        "bbox_scale": log.bbox_scale_px[index],
        "bbox_growth": log.bbox_growth_px_s[index],
        "raw_bbox_growth": log.raw_bbox_growth_px_s[index],
        "bbox_dx": log.bbox_center_dx_px[index] if log.bbox_center_dx_px else float("nan"),
        "bbox_dy": log.bbox_center_dy_px[index] if log.bbox_center_dy_px else float("nan"),
        "barometer_raw": log.barometer_raw_altitude_m[index],
        "barometer_filtered": log.barometer_filtered_altitude_m[index],
    }
