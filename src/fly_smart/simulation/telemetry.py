"""Telemetry storage and live/final trajectory plots."""

from dataclasses import dataclass, field
import csv
import json
from math import degrees, isfinite, radians
from pathlib import Path

from .drone_model import PhysicsStep

from ..camera_geometry import bbox_alignment_angles
from .config import SceneConfig, StrikeConfig
from ..guidance import GuidanceCommand
from ..sensing import BarometerReading
from ..ttc import TtcObservation


@dataclass
class FlightLog:
    time_s: list[float] = field(default_factory=list)
    x_m: list[float] = field(default_factory=list)
    z_m: list[float] = field(default_factory=list)
    y_m: list[float] = field(default_factory=list)
    vx_mps: list[float] = field(default_factory=list)
    vz_mps: list[float] = field(default_factory=list)
    barometer_raw_altitude_m: list[float] = field(default_factory=list)
    barometer_filtered_altitude_m: list[float] = field(default_factory=list)
    barometer_filtered_vertical_velocity_mps: list[float] = field(default_factory=list)
    command_vx_mps: list[float] = field(default_factory=list)
    command_vz_mps: list[float] = field(default_factory=list)
    pid_vz_target_mps: list[float] = field(default_factory=list)
    command_altitude_m: list[float] = field(default_factory=list)
    vertical_control_mode: list[str] = field(default_factory=list)
    camera_dy_correction_mps: list[float] = field(default_factory=list)
    path_slope_target_m_per_m: list[float] = field(default_factory=list)
    measured_path_slope_m_per_m: list[float] = field(default_factory=list)
    command_thrust_n: list[float] = field(default_factory=list)
    command_pitch_deg: list[float] = field(default_factory=list)
    pitch_error_deg: list[float] = field(default_factory=list)
    pitch_torque: list[float] = field(default_factory=list)
    phase: list[str] = field(default_factory=list)
    measured_pitch_deg: list[float] = field(default_factory=list)
    ttc_s: list[float] = field(default_factory=list)
    raw_ttc_s: list[float] = field(default_factory=list)
    bbox_scale_px: list[float] = field(default_factory=list)
    bbox_growth_px_s: list[float] = field(default_factory=list)
    raw_bbox_growth_px_s: list[float] = field(default_factory=list)
    bbox_center_dx_px: list[float] = field(default_factory=list)
    bbox_center_dy_px: list[float] = field(default_factory=list)
    bbox_center_dx_deg: list[float] = field(default_factory=list)
    bbox_center_dy_deg: list[float] = field(default_factory=list)
    pitch_compensated_dy_deg: list[float] = field(default_factory=list)
    body_drag_x_n: list[float] = field(default_factory=list)
    body_drag_z_n: list[float] = field(default_factory=list)
    angular_damping_pitch_torque_nm: list[float] = field(default_factory=list)
    gyroscopic_pitch_torque_nm: list[float] = field(default_factory=list)
    ground_effect_max_multiplier: list[float] = field(default_factory=list)
    collision_time_s: float | None = None
    collision_position_m: tuple[float, float, float] | None = None
    collision_velocity_mps: tuple[float, float, float] | None = None

    def append(self, now_s: float, position: tuple[float, float, float], velocity: tuple[float, float, float], command: GuidanceCommand, measured_pitch_rad: float = 0.0, pitch_torque: float = 0.0, observation: TtcObservation | None = None, physics_step: PhysicsStep | None = None, barometer: BarometerReading | None = None, camera_width_px: int = 640, camera_height_px: int = 480, camera_fov_deg: float = 90.0) -> None:
        """Record state, guidance, and optional applied shared-force telemetry."""
        self.time_s.append(now_s)
        self.x_m.append(position[0])
        self.z_m.append(position[2])
        self.y_m.append(position[1])
        self.vx_mps.append(velocity[0])
        self.vz_mps.append(velocity[2])
        self.barometer_raw_altitude_m.append(barometer.raw_altitude_m if barometer and barometer.raw_altitude_m is not None else float("nan"))
        self.barometer_filtered_altitude_m.append(barometer.altitude_m if barometer else float("nan"))
        self.barometer_filtered_vertical_velocity_mps.append(barometer.vertical_velocity_mps if barometer else float("nan"))
        trajectory = command.trajectory
        self.command_vx_mps.append(trajectory.forward_velocity_mps if trajectory else float("nan"))
        self.command_vz_mps.append(trajectory.vertical_velocity_mps if trajectory else float("nan"))
        self.pid_vz_target_mps.append(command.vertical_velocity_target_mps if command.vertical_velocity_target_mps is not None else float("nan"))
        self.command_altitude_m.append(trajectory.altitude_target_m if trajectory else float("nan"))
        self.vertical_control_mode.append(command.vertical_control_mode.value)
        self.camera_dy_correction_mps.append(command.camera_dy_correction_mps)
        self.path_slope_target_m_per_m.append(command.path_slope_target_m_per_m if command.path_slope_target_m_per_m is not None else float("nan"))
        self.measured_path_slope_m_per_m.append(velocity[2] / velocity[0] if abs(velocity[0]) >= 0.1 else float("nan"))
        self.command_thrust_n.append(command.thrust_n)
        self.command_pitch_deg.append(degrees(command.pitch_target_rad))
        self.phase.append(command.phase.value)
        self.measured_pitch_deg.append(degrees(measured_pitch_rad))
        self.pitch_error_deg.append(degrees(command.pitch_target_rad - measured_pitch_rad))
        self.pitch_torque.append(pitch_torque)
        self.ttc_s.append(observation.ttc_s if observation else float("nan"))
        self.raw_ttc_s.append(observation.raw_ttc_s if observation else float("nan"))
        self.bbox_scale_px.append(observation.scale_px if observation else float("nan"))
        self.bbox_growth_px_s.append(observation.scale_growth_px_s if observation else float("nan"))
        self.raw_bbox_growth_px_s.append(observation.raw_growth_px_s if observation else float("nan"))
        if observation:
            x, y, width, height = observation.box
            self.bbox_center_dx_px.append(x + width / 2.0 - camera_width_px / 2.0)
            self.bbox_center_dy_px.append(y + height / 2.0 - camera_height_px / 2.0)
        else:
            self.bbox_center_dx_px.append(float("nan"))
            self.bbox_center_dy_px.append(float("nan"))
        dx_deg, dy_deg, compensated_dy_deg = bbox_alignment_angles(
            observation.box if observation else None,
            camera_width_px,
            camera_height_px,
            camera_fov_deg,
            measured_pitch_rad,
        )
        self.bbox_center_dx_deg.append(dx_deg)
        self.bbox_center_dy_deg.append(dy_deg)
        self.pitch_compensated_dy_deg.append(compensated_dy_deg)
        body_drag = physics_step.body_drag_force_body_n if physics_step else (float("nan"),) * 3
        angular_damping = physics_step.angular_damping_torque_body_nm if physics_step else (float("nan"),) * 3
        gyroscopic = physics_step.gyroscopic_torque_body_nm if physics_step else (float("nan"),) * 3
        self.body_drag_x_n.append(body_drag[0])
        self.body_drag_z_n.append(body_drag[2])
        self.angular_damping_pitch_torque_nm.append(angular_damping[1])
        self.gyroscopic_pitch_torque_nm.append(gyroscopic[1])
        self.ground_effect_max_multiplier.append(max(physics_step.ground_effect_multipliers) if physics_step else float("nan"))

    def mark_collision(self, now_s: float, position: tuple[float, float, float], velocity: tuple[float, float, float]) -> None:
        if self.collision_time_s is None:
            self.collision_time_s = now_s
            self.collision_position_m = tuple(position)
            self.collision_velocity_mps = tuple(velocity)


def append_telemetry_sample(log: FlightLog, sample: dict[str, object]) -> None:
    """Append one serialized live-plot sample to a child-process log."""
    def number(name: str) -> float:
        value = sample.get(name)
        return float(value) if isinstance(value, (int, float)) else float("nan")

    log.time_s.append(number("t"))
    log.phase.append(str(sample.get("phase", "")))
    log.x_m.append(number("x"))
    log.z_m.append(number("z"))
    log.y_m.append(float("nan"))
    log.vx_mps.append(number("vx"))
    log.vz_mps.append(number("vz"))
    log.command_vx_mps.append(number("command_vx"))
    log.command_vz_mps.append(number("command_vz"))
    log.pid_vz_target_mps.append(number("pid_vz"))
    log.command_altitude_m.append(number("command_altitude"))
    log.vertical_control_mode.append(str(sample.get("vertical_control_mode", "")))
    log.camera_dy_correction_mps.append(number("camera_dy_correction"))
    log.path_slope_target_m_per_m.append(number("path_slope_target"))
    log.measured_path_slope_m_per_m.append(number("measured_path_slope"))
    log.command_thrust_n.append(number("thrust"))
    log.command_pitch_deg.append(number("pitch"))
    log.measured_pitch_deg.append(number("measured_pitch"))
    log.ttc_s.append(number("ttc"))
    log.raw_ttc_s.append(number("raw_ttc"))
    log.bbox_scale_px.append(number("bbox_scale"))
    log.bbox_growth_px_s.append(number("bbox_growth"))
    log.raw_bbox_growth_px_s.append(number("raw_bbox_growth"))
    log.bbox_center_dx_px.append(number("bbox_dx"))
    log.bbox_center_dy_px.append(number("bbox_dy"))
    log.bbox_center_dx_deg.append(number("bbox_dx_deg"))
    log.bbox_center_dy_deg.append(number("bbox_dy_deg"))
    log.pitch_compensated_dy_deg.append(number("pitch_compensated_dy_deg"))
    log.barometer_raw_altitude_m.append(number("barometer_raw"))
    log.barometer_filtered_altitude_m.append(number("barometer_filtered"))
    log.barometer_filtered_vertical_velocity_mps.append(float("nan"))
    log.pitch_error_deg.append(float("nan"))
    log.pitch_torque.append(float("nan"))
    log.body_drag_x_n.append(float("nan"))
    log.body_drag_z_n.append(float("nan"))
    log.angular_damping_pitch_torque_nm.append(float("nan"))
    log.gyroscopic_pitch_torque_nm.append(float("nan"))
    log.ground_effect_max_multiplier.append(float("nan"))


@dataclass
class TelemetryPlot:
    figure: object
    velocity_axis: object
    path_axis: object
    path_slope_axis: object
    trajectory_axis: object
    trajectory_altitude_axis: object
    guidance_axis: object
    pitch_axis: object
    growth_axis: object
    ttc_axis: object
    alignment_axis: object
    alignment_angle_axis: object
    camera_width_px: float
    camera_height_px: float
    camera_fov_rad: float
    barometer_axis: object
    target_marker: object
    ttc_gate_line: object
    growth_gate_line: object
    ttc_activation_s: float
    alignment_zero_px: object
    alignment_zero_angle: object
    lines: tuple[object, ...]
    phase_axes: tuple[object, ...]
    phase_artists: list[object] = field(default_factory=list)
    collision_axes: tuple[object, ...] = ()
    collision_artists: list[object] = field(default_factory=list)
    phase_signature: object = None


def make_plot(config: StrikeConfig, scene: SceneConfig, scenario_name: str = "default") -> TelemetryPlot:
    """Create telemetry plots labelled with the selected drone and scenario."""
    import matplotlib.pyplot as plt

    figure, (velocity_axis, path_axis, trajectory_axis, guidance_axis, growth_axis, ttc_axis, alignment_axis, barometer_axis) = plt.subplots(8, 1, figsize=(10, 20))
    figure.suptitle(f"TTC strike telemetry — {scene.drone_profile} / {scenario_name}")
    vx_line, = velocity_axis.plot([], [], label="vx measured", color="#2563eb")
    velocity_command_line, = velocity_axis.plot([], [], "--", label="vx target", color="#2563eb")
    vz_line, = velocity_axis.plot([], [], label="vz vertical", color="#dc2626")
    velocity_axis.set(xlabel="time (s)", ylabel="velocity (m/s)", title="Measured world-frame velocity")
    velocity_axis.grid(alpha=0.25)
    velocity_axis.legend()

    path_line, = path_axis.plot([], [], color="#16a34a", label="drone path")
    tracking_path_line, = path_axis.plot([], [], color="#2563eb", linewidth=2.5, label="tracking segment")
    path_slope_axis = path_axis.twinx()
    target_slope_line, = path_slope_axis.plot([], [], "--", color="#f97316", label="target slope")
    measured_slope_line, = path_slope_axis.plot([], [], ":", color="#7c3aed", label="measured slope")
    target_marker = path_axis.scatter((scene.target_center[0],), (scene.target_center[2],), color="#dc2626", label="scene target")
    path_axis.set(xlabel="world x (m)", ylabel="world z / altitude (m)", title="Measured diagonal path (x-z)")
    path_slope_axis.set_ylabel("path slope dz/dx (m/m)")
    path_axis.grid(alpha=0.25)
    path_axis.legend((path_line, tracking_path_line, target_slope_line, measured_slope_line), ("drone path", "tracking segment", "target slope", "measured slope"), loc="upper left")

    command_vx_line, = trajectory_axis.plot([], [], label="command vx", color="#2563eb")
    command_vz_line, = trajectory_axis.plot([], [], label="TTC vz", color="#dc2626")
    pid_vz_target_line, = trajectory_axis.plot([], [], label="PID vz target", color="#7c3aed")
    camera_dy_correction_line, = trajectory_axis.plot([], [], "-.", label="camera dy correction (DY mode)", color="#0891b2")
    trajectory_altitude_axis = trajectory_axis.twinx()
    command_altitude_line, = trajectory_altitude_axis.plot([], [], "--", label="altitude target", color="#16a34a")
    trajectory_axis.set(xlabel="time (s)", ylabel="velocity command (m/s)", title="TrajectoryCommand")
    trajectory_altitude_axis.set_ylabel("altitude target (m)")
    trajectory_axis.grid(alpha=0.25)
    trajectory_axis.legend((command_vx_line, command_vz_line, pid_vz_target_line, camera_dy_correction_line, command_altitude_line), ("command vx", "TTC vz", "PID vz target", "camera dy correction (DY mode)", "altitude target"), loc="upper left")

    thrust_line, = guidance_axis.plot([], [], label="collective thrust", color="#7c3aed")
    pitch_axis = guidance_axis.twinx()
    pitch_line, = pitch_axis.plot([], [], label="pitch target", color="#f97316")
    measured_pitch_line, = pitch_axis.plot([], [], "--", label="pitch measured", color="#ea580c")
    guidance_axis.set(xlabel="time (s)", ylabel="GuidanceCommand thrust (N)", title="GuidanceCommand")
    pitch_axis.set_ylabel("pitch target (deg)")
    guidance_axis.grid(alpha=0.25)
    guidance_axis.legend((thrust_line, pitch_line, measured_pitch_line), ("collective thrust", "pitch target", "pitch measured"), loc="upper left")

    raw_growth_line, = growth_axis.plot([], [], "--", color="#f97316", alpha=0.8, label="raw bbox growth")
    filtered_growth_line, = growth_axis.plot([], [], color="#2563eb", linewidth=2, label="alpha-beta estimated growth")
    growth_gate_line = growth_axis.axvline(float("nan"), color="#b45309", linestyle=":", label="first valid growth")
    growth_axis.set(xlabel="time (s)", ylabel="growth (px/s)", title="Alpha-beta bbox growth filter")
    growth_axis.grid(alpha=0.25)
    growth_axis.legend()

    raw_ttc_line, = ttc_axis.plot([], [], "--", color="#f97316", alpha=0.8, label="raw TTC")
    filtered_ttc_line, = ttc_axis.plot([], [], color="#2563eb", linewidth=2, label="filtered TTC")
    ttc_gate_line = ttc_axis.axvline(float("nan"), color="#b45309", linestyle=":", label=f"TTC activation ({config.ttc_activation_s:g} s)")
    ttc_axis.set(xlabel="time (s)", ylabel="TTC (s)", title="Time-to-contact estimate")
    ttc_axis.grid(alpha=0.25)
    ttc_axis.legend()

    alignment_angle_axis = alignment_axis.twinx()
    dx_line, = alignment_axis.plot([], [], color="#2563eb", label="dx pixel")
    dy_line, = alignment_axis.plot([], [], color="#dc2626", label="dy pixel")
    dx_angle_line, = alignment_angle_axis.plot([], [], "--", color="#2563eb", alpha=0.75, label="dx angle")
    dy_angle_line, = alignment_angle_axis.plot([], [], "--", color="#dc2626", alpha=0.75, label="dy angle")
    compensated_dy_line, = alignment_angle_axis.plot([], [], "-.", color="#7c3aed", alpha=0.9, label="dy angle − pitch")
    alignment_zero_px = alignment_axis.axhline(0.0, color="#374151", linestyle=":")
    alignment_zero_angle = alignment_angle_axis.axhline(0.0, color="#374151", linestyle=":")
    alignment_axis.set(xlabel="time (s)", ylabel="center error (px)", title="Target alignment")
    alignment_angle_axis.set_ylabel("center error (deg)")
    alignment_axis.grid(alpha=0.25)
    alignment_axis.legend((dx_line, dy_line, dx_angle_line, dy_angle_line, compensated_dy_line), ("dx pixel", "dy pixel", "dx angle", "dy angle", "dy angle − pitch"), loc="upper left")

    raw_altitude_line, = barometer_axis.plot([], [], "--", color="#f97316", alpha=0.8, label="raw barometer altitude")
    filtered_altitude_line, = barometer_axis.plot([], [], color="#2563eb", linewidth=2, label="filtered barometer altitude")
    true_altitude_line, = barometer_axis.plot([], [], ":", color="#16a34a", label="true PyBullet altitude")
    barometer_axis.set(xlabel="time (s)", ylabel="altitude (m)", title="BMP388 altitude EMA filter")
    barometer_axis.grid(alpha=0.25)
    barometer_axis.legend()
    figure.tight_layout(rect=(0, 0, 1, 0.98))
    return TelemetryPlot(
        figure,
        velocity_axis,
        path_axis,
        path_slope_axis,
        trajectory_axis,
        trajectory_altitude_axis,
        guidance_axis,
        pitch_axis,
        growth_axis,
        ttc_axis,
        alignment_axis,
        alignment_angle_axis,
        float(config.camera_width_px),
        float(config.camera_height_px),
        radians(config.camera_fov_deg),
        barometer_axis,
        target_marker,
        ttc_gate_line,
        growth_gate_line,
        config.ttc_activation_s,
        alignment_zero_px,
        alignment_zero_angle,
        (vx_line, velocity_command_line, vz_line, path_line, tracking_path_line, target_slope_line, measured_slope_line, command_vx_line, command_vz_line, pid_vz_target_line, camera_dy_correction_line, command_altitude_line, thrust_line, pitch_line, measured_pitch_line, raw_growth_line, filtered_growth_line, raw_ttc_line, filtered_ttc_line, dx_line, dy_line, dx_angle_line, dy_angle_line, compensated_dy_line, raw_altitude_line, filtered_altitude_line, true_altitude_line),
        (velocity_axis, guidance_axis, growth_axis, ttc_axis, alignment_axis, alignment_angle_axis, barometer_axis),
        collision_axes=(velocity_axis, trajectory_axis, guidance_axis, growth_axis, ttc_axis, alignment_axis, alignment_angle_axis, barometer_axis),
    )


def _phase_intervals(log: FlightLog) -> list[tuple[str, float, float]]:
    """Return contiguous phase intervals using the recorded sample times."""
    if not log.time_s:
        return []
    intervals: list[tuple[str, float, float]] = []
    start, phase = log.time_s[0], log.phase[0]
    for index in range(1, len(log.time_s)):
        if log.phase[index] != phase:
            intervals.append((phase, start, log.time_s[index - 1]))
            start, phase = log.time_s[index], log.phase[index]
    intervals.append((phase, start, log.time_s[-1]))
    return intervals


def _refresh_phase_backgrounds(plot: TelemetryPlot, log: FlightLog) -> None:
    """Shade the tracking interval and keep old live-plot patches bounded."""
    signature = (tuple(log.phase[-20:]), log.collision_time_s)
    if signature == plot.phase_signature:
        return
    plot.phase_signature = signature
    for artist in plot.phase_artists:
        artist.remove()
    plot.phase_artists.clear()
    end_limit = log.collision_time_s if log.collision_time_s is not None else (log.time_s[-1] if log.time_s else None)
    if end_limit is None:
        return
    for phase, start, end in _phase_intervals(log):
        if phase != "track":
            continue
        if start >= end_limit:
            continue
        end = min(end, end_limit)
        for axis in plot.phase_axes:
            plot.phase_artists.append(axis.axvspan(start, end, color="#bfdbfe", alpha=0.28, zorder=0))
    for artist in plot.collision_artists:
        artist.remove()
    plot.collision_artists.clear()
    if log.collision_time_s is not None:
        for axis in plot.collision_axes:
            plot.collision_artists.append(axis.axvline(log.collision_time_s, color="#b45309", linestyle="--", linewidth=1.2, label="collision"))


def _first_time(values: list[float], times: list[float], predicate) -> float | None:
    """Return the first sample time satisfying a display gate."""
    for time, value in zip(times, values):
        if isfinite(value) and predicate(value):
            return time
    return None


def _growth_display_ceiling(*series: list[float]) -> float:
    """Return a robust display ceiling without changing recorded telemetry."""
    values = sorted(abs(value) for samples in series for value in samples if isfinite(value))
    if not values:
        return 1.0
    index = len(values) // 2 if len(values) < 20 else int(0.95 * (len(values) - 1))
    return max(1.0, values[max(0, index)])


def _set_gate_line(line: object, time_s: float | None) -> None:
    """Show one vertical gate marker or keep it hidden when no gate was reached."""
    value = float("nan") if time_s is None else time_s
    line.set_xdata([value, value])


def refresh_plot(plot: TelemetryPlot, log: FlightLog, autoscale: bool = True, max_points: int | None = None) -> None:
    """Update the figure, optionally limiting rendered history and rescaling."""
    vx_line, velocity_command_line, vz_line, path_line, tracking_path_line, target_slope_line, measured_slope_line, command_vx_line, command_vz_line, pid_vz_target_line, camera_dy_correction_line, command_altitude_line, thrust_line, pitch_line, measured_pitch_line, raw_growth_line, filtered_growth_line, raw_ttc_line, filtered_ttc_line, dx_line, dy_line, dx_angle_line, dy_angle_line, compensated_dy_line, raw_altitude_line, filtered_altitude_line, true_altitude_line = plot.lines
    start = max(0, len(log.time_s) - max_points) if max_points else 0
    times = log.time_s[start:]
    phases = log.phase[start:]
    def recent(values: list[object]) -> list[object]:
        return values[start:]

    vx_line.set_data(times, recent(log.vx_mps))
    velocity_command_line.set_data(times, recent(log.command_vx_mps))
    vz_line.set_data(times, recent(log.vz_mps))
    path_line.set_data(recent(log.x_m), recent(log.z_m))
    target_slope_line.set_data(recent(log.x_m), recent(log.path_slope_target_m_per_m))
    measured_slope_line.set_data(recent(log.x_m), recent(log.measured_path_slope_m_per_m))
    tracking_path_line.set_data(
        [x if phase == "track" and (log.collision_time_s is None or time <= log.collision_time_s) else float("nan") for x, phase, time in zip(recent(log.x_m), phases, times)],
        [z if phase == "track" and (log.collision_time_s is None or time <= log.collision_time_s) else float("nan") for z, phase, time in zip(recent(log.z_m), phases, times)],
    )
    command_vx_line.set_data(times, recent(log.command_vx_mps))
    command_vz_line.set_data(times, recent(log.command_vz_mps))
    pid_vz_target_line.set_data(times, recent(log.pid_vz_target_mps))
    command_altitude_line.set_data(times, recent(log.command_altitude_m))
    thrust_line.set_data(times, recent(log.command_thrust_n))
    pitch_line.set_data(times, recent(log.command_pitch_deg))
    measured_pitch_line.set_data(times, recent(log.measured_pitch_deg))
    tracking = [phase == "track" and (log.collision_time_s is None or time <= log.collision_time_s) for phase, time in zip(phases, times)]
    def tracking_values(values: list[float]) -> list[float]:
        return [value if active and isfinite(value) else float("nan") for value, active in zip(values, tracking)]

    camera_dy_correction_line.set_data(times, tracking_values(recent(log.camera_dy_correction_mps)))
    growth_ceiling = _growth_display_ceiling(log.raw_bbox_growth_px_s, log.bbox_growth_px_s)
    growth_values = lambda values: [value if active and isfinite(value) and abs(value) <= growth_ceiling else float("nan") for value, active in zip(values, tracking)]
    raw_growth_line.set_data(times, growth_values(recent(log.raw_bbox_growth_px_s)))
    filtered_growth_line.set_data(times, growth_values(recent(log.bbox_growth_px_s)))
    ttc_gate_time = _first_time(log.ttc_s, log.time_s, lambda value: value <= plot.ttc_activation_s)
    ttc_values = lambda values: [value if active and isfinite(value) and ttc_gate_time is not None and time >= ttc_gate_time else float("nan") for value, active, time in zip(values, tracking, times)]
    raw_ttc_line.set_data(times, ttc_values(recent(log.raw_ttc_s)))
    filtered_ttc_line.set_data(times, ttc_values(recent(log.ttc_s)))
    _set_gate_line(plot.ttc_gate_line, ttc_gate_time)
    _set_gate_line(plot.growth_gate_line, _first_time(log.bbox_growth_px_s, log.time_s, lambda value: True))
    dx_line.set_data(times, tracking_values(recent(log.bbox_center_dx_px)))
    dy_line.set_data(times, tracking_values(recent(log.bbox_center_dy_px)))
    dx_angle_line.set_data(times, tracking_values(recent(log.bbox_center_dx_deg)))
    dy_angle_line.set_data(times, tracking_values(recent(log.bbox_center_dy_deg)))
    compensated_dy_line.set_data(times, tracking_values(recent(log.pitch_compensated_dy_deg)))
    raw_altitude_line.set_data(times, recent(log.barometer_raw_altitude_m))
    filtered_altitude_line.set_data(times, recent(log.barometer_filtered_altitude_m))
    true_altitude_line.set_data(times, recent(log.z_m))
    _refresh_phase_backgrounds(plot, log)
    if autoscale:
        for axis in (plot.velocity_axis, plot.path_axis, plot.path_slope_axis, plot.trajectory_axis, plot.trajectory_altitude_axis, plot.guidance_axis, plot.pitch_axis, plot.growth_axis, plot.ttc_axis, plot.alignment_axis, plot.alignment_angle_axis, plot.barometer_axis):
            axis.relim()
            axis.autoscale_view()
    plot.figure.canvas.draw_idle()
    plot.figure.canvas.flush_events()


def move_plot_window(plot: TelemetryPlot, position_px: tuple[int, int]) -> None:
    """Place the Matplotlib window on Tk or Qt without coupling to one backend."""
    window = plot.figure.canvas.manager.window
    x, y = position_px
    if hasattr(window, "wm_geometry"):  # TkAgg
        window.wm_geometry(f"+{x}+{y}")
    elif hasattr(window, "move"):  # QtAgg
        window.move(x, y)


def save_plot(log: FlightLog, config: StrikeConfig, scene: SceneConfig, output: Path, scenario_name: str = "default") -> None:
    import matplotlib.pyplot as plt

    output.parent.mkdir(parents=True, exist_ok=True)
    plot = make_plot(config, scene, scenario_name)
    refresh_plot(plot, log)
    plot.figure.savefig(output, dpi=140)
    plt.close(plot.figure)


def save_csv(log: FlightLog, output: Path) -> None:
    """Write measured state and every high-level command for offline tuning."""
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = ("time_s", "phase", "x_m", "y_m", "z_m", "vx_mps", "vz_mps", "barometer_raw_altitude_m", "barometer_filtered_altitude_m", "barometer_filtered_vertical_velocity_mps", "command_vx_mps",
              "command_vz_mps", "pid_vz_target_mps", "command_altitude_m", "vertical_control_mode", "camera_dy_correction_mps", "path_slope_target_m_per_m", "measured_path_slope_m_per_m", "command_thrust_n", "command_pitch_deg",
              "measured_pitch_deg", "pitch_error_deg", "pitch_torque", "ttc_s", "raw_ttc_s", "bbox_scale_px", "bbox_growth_px_s", "raw_bbox_growth_px_s", "bbox_center_dx_px", "bbox_center_dy_px", "bbox_center_dx_deg", "bbox_center_dy_deg", "pitch_compensated_dy_deg",
              "body_drag_x_n", "body_drag_z_n", "angular_damping_pitch_torque_nm", "gyroscopic_pitch_torque_nm", "ground_effect_max_multiplier")
    with output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(zip(*(getattr(log, field) for field in fields)))


def build_summary(log: FlightLog, config: StrikeConfig, scene: SceneConfig, success: bool, final_phase: str, simulated_time_s: float, outputs: dict[str, Path | None], abort_reason: str | None = None) -> dict[str, object]:
    """Build JSON-safe scene and collision metrics for one completed run."""
    def finite(values: list[float]) -> list[float]:
        return [value for value in values if value == value]

    intervals = []
    for phase, start, end in _phase_intervals(log):
        if log.collision_time_s is not None:
            if start >= log.collision_time_s:
                continue
            end = min(end, log.collision_time_s)
        intervals.append({"phase": phase, "start_s": start, "end_s": end})
    if log.collision_time_s is not None and log.collision_time_s < simulated_time_s:
        intervals.append({"phase": "post-impact", "start_s": log.collision_time_s, "end_s": simulated_time_s})
    speed_values = [abs(value) for value in log.vx_mps]
    vertical_values = [abs(value) for value in log.vz_mps]
    ttc_values = finite(log.ttc_s)
    collision_velocity = log.collision_velocity_mps
    return {
        "success": success,
        "final_phase": final_phase,
        "abort_reason": abort_reason,
        "simulated_time_s": simulated_time_s,
        "starting_pose": {"position_m": list(config.launch_position), "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "target": {"center_m": list(scene.target_center), "size_m": scene.target_size_m},
        "collision": {
            "time_s": log.collision_time_s,
            "position_m": list(log.collision_position_m) if log.collision_position_m else None,
            "velocity_mps": list(collision_velocity) if collision_velocity else None,
            "speed_mps": sum(component * component for component in collision_velocity) ** 0.5 if collision_velocity else None,
        },
        "flight_metrics": {
            "maximum_altitude_m": max(log.z_m) if log.z_m else None,
            "minimum_altitude_m": min(log.z_m) if log.z_m else None,
            "maximum_forward_speed_mps": max(speed_values) if speed_values else None,
            "maximum_vertical_speed_mps": max(vertical_values) if vertical_values else None,
            "last_ttc_s": ttc_values[-1] if ttc_values else None,
        },
        "phase_intervals": intervals,
        "outputs": {key: str(value) if value else None for key, value in outputs.items()},
    }


def save_summary(summary: dict[str, object], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2) + "\n")
