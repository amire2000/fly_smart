"""PyBullet adapter that composes sensing, TTC, guidance, views, and telemetry."""

from dataclasses import dataclass, replace
from math import degrees, sqrt
from pathlib import Path
import time

import cv2
import pybullet as p

from .drone_physics import PhysicsEngine, clamp
from ..common.flight_control import AttitudeController
from .pybullet_sensors import read_imu
from .pybullet_utils import create_world, draw_force_vectors, reset_drone
from .gui_helper import SimulationControls
from .forward_camera import add_environment_buildings, add_red_cube, forward_rgb
from .godot_bridge import GodotBridge
from ..red_target_detector import detect_red_box

from .config import SceneConfig, StrikeConfig
from ..camera_geometry import bbox_alignment_angles
from ..guidance import FlightPhase, GuidanceCommand, GuidanceInput, StrikeGuidance
from ..sensing import BarometerReading, VerticalEstimator
from .sensors import Barometer, VerticalImu
from .telemetry import FlightLog, build_summary, save_csv, save_plot, save_summary
from .plot_process import latest_sample, send_plot_message, start_plot_process
from ..ttc import BboxTtcTracker, TtcObservation
from .views import annotate, environment_rgb


TARGET_DISTANCE_PRESETS_M = (30.0, 40.0, 50.0, 60.0, 70.0, 80.0)


def real_time_factor(simulated_seconds: float, wall_seconds: float) -> float:
    """Return simulated time divided by elapsed wall-clock time."""
    return simulated_seconds / wall_seconds if wall_seconds > 0.0 else 0.0


def pacing_sleep_seconds(deadline: float, now: float) -> float:
    """Return only the remaining time before a real-time deadline."""
    return max(0.0, deadline - now)


def pace_until(deadline: float) -> None:
    """Sleep coarsely, then spin briefly to avoid scheduler overshoot."""
    while True:
        remaining = deadline - time.perf_counter()
        if remaining <= 0.0:
            return
        if remaining > 0.001:
            time.sleep(remaining - 0.001)


@dataclass(frozen=True)
class StrikeResult:
    success: bool
    phase: str
    simulated_time_s: float
    impact_speed_mps: float
    video: Path | None
    plot: Path | None
    csv: Path | None
    summary: Path | None


class StrikeSimulation:
    """Run one configured strike against the concrete PyBullet simulator."""

    def __init__(self, config: StrikeConfig | None = None, scene: SceneConfig | None = None, godot: GodotBridge | None = None, scenario_name: str = "default") -> None:
        self.config = config or StrikeConfig()
        self.scene = scene or self.config.simulation
        self.godot = godot
        self.scenario_name = scenario_name

    def run(self, gui: bool, max_seconds: float, video: Path | None, plot: Path | None, csv: Path | None = None, summary: Path | None = None, show_plots: bool = False, interactive: bool = False) -> StrikeResult:
        config = self.config
        model = self.scene.drone_model
        settings = self.scene.physics_settings
        physics_hz = settings.physics_hz
        time_step = settings.time_step_s
        control_steps = settings.control_steps
        active_scene = self.scene
        active_target_distance_m = self.scene.target_center[0] - config.launch_position[0]
        drone = create_world(model, settings)
        engine = PhysicsEngine(model, settings)
        p.resetBasePositionAndOrientation(drone, config.launch_position, (0, 0, 0, 1))
        cube = add_red_cube(active_scene.target_center, active_scene.target_size_m, collision=self.godot is None)
        add_environment_buildings()
        if self.godot:
            self.godot.open()
        barometer, tracker, guidance = Barometer(config), BboxTtcTracker(config), StrikeGuidance(config)
        vertical_imu = VerticalImu(config)
        vertical_estimator = VerticalEstimator(config, config.launch_position[2])
        previous_vertical_velocity_mps = 0.0
        attitude_controller = AttitudeController(config.pitch_attitude_pid_gains)
        torque = (0.0, 0.0, 0.0)
        command = GuidanceCommand(FlightPhase.TAKEOFF, config.hover_thrust_n, 0.0, None)
        baro = BarometerReading(config.launch_position[2], 0.0)
        observation: TtcObservation | None = None
        target_visible = False
        vertical_alignment_error_deg: float | None = None
        force_lines = [-1, -1, -1, -1]
        renderer = p.ER_BULLET_HARDWARE_OPENGL if gui else p.ER_TINY_RENDERER
        impact_speed, stop_at_s = 0.0, None
        rtf_started: float | None = None
        simulated_elapsed_s = 0.0
        log = FlightLog()
        writer = None
        attempt_number = 0
        attempt_video, attempt_plot, attempt_csv, attempt_summary = video, plot, csv, summary
        plot_handle = start_plot_process(config, active_scene, self.scenario_name, plot) if gui or show_plots or interactive else None
        plot_process, plot_sender = plot_handle if plot_handle else (None, None)
        next_plot_emit = time.perf_counter()
        controls: SimulationControls | None = SimulationControls() if interactive else None
        show_frame = gui
        if show_frame:
            cv2.namedWindow("TTC diagonal strike", cv2.WINDOW_NORMAL)
            cv2.moveWindow("TTC diagonal strike", *config.opencv_window_position_px)
        if gui:
            p.resetDebugVisualizerCamera(36.0, 48.0, -25.0, (7.0, 0.0, 7.0))

        def current_rtf() -> float:
            return real_time_factor(simulated_elapsed_s, running_time_seconds())

        def running_time_seconds() -> float:
            """Return active wall-clock seconds for the current attempt."""
            return time.perf_counter() - rtf_started if rtf_started is not None else 0.0

        def reset_attempt() -> None:
            """Restore the complete flight state and publish Godot's initial pose."""
            nonlocal attempt_number, attempt_video, attempt_plot, attempt_csv, attempt_summary
            nonlocal engine, barometer, tracker, guidance, vertical_imu, vertical_estimator
            nonlocal previous_vertical_velocity_mps, attitude_controller, torque, command, baro
            nonlocal observation, target_visible, vertical_alignment_error_deg, impact_speed, stop_at_s, log, writer, rtf_started, simulated_elapsed_s
            if writer:
                writer.release()
            if interactive and attempt_number:
                if attempt_plot:
                    save_plot(log, config, active_scene, attempt_plot, self.scenario_name)
                if attempt_csv:
                    save_csv(log, attempt_csv)
                if attempt_summary:
                    save_summary(
                        build_summary(
                            log,
                            config,
                            active_scene,
                            False,
                            command.phase.value,
                            log.time_s[-1] if log.time_s else 0.0,
                            {"video": attempt_video, "plot": attempt_plot, "csv": attempt_csv, "summary": attempt_summary},
                            "Restarted",
                        ),
                        attempt_summary,
                    )
            attempt_number += 1
            if interactive and summary:
                attempt_dir = summary.parent / f"attempt-{attempt_number:03d}"
                attempt_video = attempt_dir / video.name if video else None
                attempt_plot = attempt_dir / plot.name if plot else None
                attempt_csv = attempt_dir / csv.name if csv else None
                attempt_summary = attempt_dir / summary.name
            writer = self._video_writer(attempt_video, config)
            reset_drone(drone, config.launch_position)
            p.resetBasePositionAndOrientation(cube, active_scene.target_center, (0, 0, 0, 1))
            p.resetBaseVelocity(cube, (0, 0, 0), (0, 0, 0))
            engine = PhysicsEngine(model, settings)
            barometer, tracker, guidance = Barometer(config), BboxTtcTracker(config), StrikeGuidance(config)
            vertical_imu = VerticalImu(config)
            vertical_estimator = VerticalEstimator(config, config.launch_position[2])
            previous_vertical_velocity_mps = 0.0
            attitude_controller = AttitudeController(config.pitch_attitude_pid_gains)
            torque = (0.0, 0.0, 0.0)
            command = GuidanceCommand(FlightPhase.TAKEOFF, config.hover_thrust_n, 0.0, None)
            baro = BarometerReading(config.launch_position[2], 0.0)
            observation, target_visible, vertical_alignment_error_deg = None, False, None
            impact_speed, stop_at_s = 0.0, None
            rtf_started = None
            simulated_elapsed_s = 0.0
            log = FlightLog()
            if plot_sender:
                send_plot_message(plot_sender, {"type": "target", "center_m": list(active_scene.target_center)})
                send_plot_message(plot_sender, {"type": "reset"})
            if self.godot:
                self.godot.clear_collision_events()
                self.godot.clear_control_events()
                target_position, target_orientation = p.getBasePositionAndOrientation(cube)
                self.godot.publish_pose(config.launch_position, (0, 0, 0, 1), target_position, target_orientation, reset=True, overlay={"bbox": None, "rtf": current_rtf(), "running_time_s": running_time_seconds()}, target_control={"distance_m": active_target_distance_m})

        reset_attempt()

        def finish(success: bool, phase: str, now_s: float, abort_reason: str | None = None) -> StrikeResult:
            if attempt_plot:
                save_plot(log, config, active_scene, attempt_plot, self.scenario_name)
            if attempt_csv:
                save_csv(log, attempt_csv)
            result = StrikeResult(success, phase, now_s, impact_speed, attempt_video, attempt_plot, attempt_csv, attempt_summary)
            summary_data = build_summary(log, config, active_scene, success, phase, now_s, {"video": attempt_video, "plot": attempt_plot, "csv": attempt_csv, "summary": attempt_summary}, abort_reason)
            if attempt_summary:
                save_summary(summary_data, attempt_summary)
            self._print_summary(result, summary_data)
            return result

        def finish_attempt(success: bool, phase: str, now_s: float, abort_reason: str | None = None) -> StrikeResult | None:
            """Finish one interactive attempt, or return its result to the caller."""
            result = finish(success, phase, now_s, abort_reason)
            if controls:
                controls.stop()
                return None
            return result

        def finish_if_disconnected(now_s: float) -> StrikeResult | None:
            """Save the partial run when closing the PyBullet GUI disconnects its server."""
            if p.isConnected():
                return None
            return finish(False, command.phase.value, now_s, "PyBullet physics server closed")

        def poll_control() -> None:
            """Apply the newest Godot toolbar command to the local state."""
            nonlocal rtf_started, active_scene, active_target_distance_m
            if not controls or not self.godot:
                return
            event = self.godot.read_control_event()
            if not event:
                return
            action = event["action"]
            selected = event.get("target_distance_m")
            if action == "reset":
                try:
                    selected = float(selected)
                except (TypeError, ValueError):
                    selected = None
                if selected in TARGET_DISTANCE_PRESETS_M:
                    active_target_distance_m = selected
                    active_scene = replace(self.scene, target_center=(config.launch_position[0] + selected, self.scene.target_center[1], self.scene.target_center[2]))
                controls.request_reset()
            elif action == "start":
                if not controls.running:
                    rtf_started = time.perf_counter()
                controls.start()
            elif action == "stop":
                controls.request_stop()

        try:
            step = 0
            next_deadline = time.perf_counter()
            max_steps = round(max_seconds / time_step)
            while step < max_steps or controls:
                if step >= max_steps:
                    print(f"Strike timed out in {command.phase.value} phase")
                    result = finish_attempt(False, command.phase.value, max_seconds)
                    if result is not None:
                        return result
                    step = 0
                    continue
                if controls:
                    while not controls.running:
                        if controls.exit_requested:
                            return finish(False, command.phase.value, step * time_step, "Interactive session closed")
                        poll_control()
                        if controls.consume_stop():
                            return finish(False, command.phase.value, step * time_step, "Stopped by operator")
                        if controls.consume_reset():
                            reset_attempt()
                            step = 0
                        time.sleep(0.02)
                        if show_frame:
                            cv2.waitKey(1)
                    poll_control()
                    if controls.consume_stop():
                        return finish(False, command.phase.value, step * time_step, "Stopped by operator")
                    if controls.consume_reset():
                        reset_attempt()
                        step = 0
                        continue
                    next_deadline = time.perf_counter()
                if controls:
                    poll_control()
                    if controls.consume_stop():
                        return finish(False, command.phase.value, step * time_step, "Stopped by operator")
                now_s = step * time_step
                disconnected = finish_if_disconnected(now_s)
                if disconnected:
                    return disconnected
                position, _ = p.getBasePositionAndOrientation(drone)
                current_vertical_velocity_mps = p.getBaseVelocity(drone)[0][2]
                imu = vertical_imu.sample((current_vertical_velocity_mps - previous_vertical_velocity_mps) / time_step, now_s)
                previous_vertical_velocity_mps = current_vertical_velocity_mps
                sample = barometer.sample(position[2], now_s)
                estimate = vertical_estimator.update(imu, time_step, sample)
                baro = BarometerReading(estimate.altitude_m, estimate.vertical_velocity_mps, sample.raw_altitude_m if sample else None)

                frame = None
                if step % (physics_hz // config.camera_hz) == 0:
                    if self.godot:
                        drone_position, drone_orientation = p.getBasePositionAndOrientation(drone)
                        target_position, target_orientation = p.getBasePositionAndOrientation(cube)
                        godot_frame = self.godot.read_frame()
                        box = None
                        if godot_frame is not None:
                            frame, box = detect_red_box(godot_frame)
                            if writer:
                                writer.write(cv2.resize(frame, config.environment_size_px))
                            target_visible = box is not None
                            observation = tracker.update(box, now_s)
                            vertical_alignment_error_deg = bbox_alignment_angles(
                                box,
                                config.camera_width_px,
                                config.camera_height_px,
                                config.camera_fov_deg,
                                p.getEulerFromQuaternion(drone_orientation)[1],
                            )[2]
                        trajectory = command.trajectory
                        self.godot.publish_pose(
                            drone_position,
                            drone_orientation,
                            target_position,
                            target_orientation,
                            overlay={
                                "bbox": list(box) if box else None,
                                "phase": command.phase.value,
                                "pitch_deg": degrees(command.pitch_target_rad),
                                "thrust_n": command.thrust_n,
                                "bbox_scale_px": observation.scale_px if observation else None,
                                "bbox_growth_px_s": observation.scale_growth_px_s if observation else None,
                                "ttc_s": observation.ttc_s if observation else None,
                                "command_vx_mps": trajectory.forward_velocity_mps if trajectory else None,
                                "command_vz_mps": trajectory.vertical_velocity_mps if trajectory else None,
                                "rtf": current_rtf(),
                                "running_time_s": running_time_seconds(),
                            },
                        )
                    else:
                        if writer:
                            writer.write(cv2.cvtColor(environment_rgb(renderer, config), cv2.COLOR_RGB2BGR))
                        frame, box = detect_red_box(
                            forward_rgb(
                                drone,
                                renderer,
                                look_down_degrees=config.camera_look_down_deg,
                                width_px=config.camera_width_px,
                                height_px=config.camera_height_px,
                                fov_deg=config.camera_fov_deg,
                            )
                        )
                        target_visible = box is not None
                        observation = tracker.update(box, now_s)
                        vertical_alignment_error_deg = bbox_alignment_angles(
                            box,
                            config.camera_width_px,
                            config.camera_height_px,
                            config.camera_fov_deg,
                            p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[1],
                        )[2]

                if step % control_steps == 0:
                    if stop_at_s is None:
                        current_velocity = p.getBaseVelocity(drone)[0]
                        measured_pitch = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[1]
                        command = guidance.update(GuidanceInput(
                            now_s,
                            baro,
                            observation,
                            tracker.last_observation,
                            target_visible,
                            tracker.commit_ready,
                            current_velocity[0],
                            measured_pitch,
                            vertical_alignment_error_deg,
                        ))
                        if command.reset_ttc:
                            # This flag belongs to the takeoff-to-track handoff:
                            # ignore bbox scale accumulated during vertical climb.
                            tracker.reset()
                            observation = None
                        if command.commit_expired:
                            # The held terminal command exceeded its predicted
                            # TTC window without contacting the cube.
                            print("Commit deadline expired without contact")
                            result = finish_attempt(False, command.phase.value, now_s)
                            if result is not None:
                                return result
                            step = 0
                            continue
                        if command.phase == FlightPhase.ABORT:
                            # Guidance has already neutralized its pitch request;
                            # stop before applying another flight-control cycle.
                            last_height = tracker.last_observation.box[3] if tracker.last_observation else 0
                            result = finish_attempt(
                                False,
                                command.phase.value,
                                now_s,
                                f"target lost before commit (last bbox height {last_height:g} px)",
                            )
                            if result is not None:
                                return result
                            step = 0
                            continue
                        # pitch_target_rad is a high-level attitude request.
                        # The attitude controller compares it with the IMU attitude and
                        # returns the body torque needed by the motor mixer.
                        torque = attitude_controller.update(read_imu(drone), yaw_target=0.0, pitch_target=command.pitch_target_rad)
                    else:
                        # Post-impact: do not keep steering or accelerating.
                        torque = (0.0, 0.0, 0.0)

                # thrust_n is the collective force. Split it evenly before
                # mapping force to a PWM signal for the four motors.
                collective = 0.0 if stop_at_s is not None else command.thrust_n
                pwm = engine.pwm_from_thrust(clamp(collective / 4, 0.0, model.max_thrust_per_motor_n))
                incoming_velocity = p.getBaseVelocity(drone)[0]
                flight_step = engine.step(drone, pwm, torque)
                if rtf_started is None:
                    rtf_started = time.perf_counter()
                simulated_elapsed_s += time_step
                disconnected = finish_if_disconnected(now_s)
                if disconnected:
                    return disconnected
                position, _ = p.getBasePositionAndOrientation(drone)
                velocity, _ = p.getBaseVelocity(drone)
                pitch_rad = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(drone)[1])[1]
                pitch_torque = torque[1]
                if stop_at_s is None:
                    # Record the collision sample, then freeze telemetry while
                    # passive post-impact physics continues for the video.
                    log.append(now_s, position, velocity, command, pitch_rad, pitch_torque, observation, flight_step, baro, config.camera_width_px, config.camera_height_px, config.camera_fov_deg)

                collision_kind = self.godot.read_collision_event() if self.godot else ("target" if p.getContactPoints(drone, cube) else None)
                if stop_at_s is None and collision_kind == "target":
                    impact_speed = sqrt(sum(component**2 for component in incoming_velocity))
                    log.mark_collision(now_s, position, incoming_velocity)
                    stop_at_s = now_s + config.post_impact_seconds
                    print(f"Impact: {impact_speed:.1f} m/s; recording aftermath for {config.post_impact_seconds:.0f} s")
                    if plot_sender:
                        send_plot_message(plot_sender, {"type": "collision", "time_s": now_s})
                elif stop_at_s is None and collision_kind == "obstacle":
                    result = finish_attempt(False, command.phase.value, now_s, "Godot obstacle collision")
                    if result is not None:
                        return result
                    step = 0
                    continue
                if plot_sender and time.perf_counter() >= next_plot_emit:
                    sample = latest_sample(log)
                    if sample:
                        send_plot_message(plot_sender, sample)
                    next_plot_emit = time.perf_counter() + 0.2
                if stop_at_s is not None and now_s >= stop_at_s:
                    # Contact is the geometry-free success condition.  Keep
                    # impact speed as telemetry instead of rejecting a valid
                    # strike because the simulated vehicle model is tuned
                    # differently from a real airframe.
                    result = finish_attempt(True, "post-impact", now_s)
                    if result is not None:
                        return result
                    step = 0
                    continue

                if show_frame:
                    if frame is not None:
                        cv2.imshow("TTC diagonal strike", annotate(frame, command, observation))
                        if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                            return finish(False, command.phase.value, now_s)
                if gui:
                    draw_force_vectors(drone, flight_step, force_lines)
                if controls and controls.consume_reset():
                    reset_attempt()
                    step = 0
                    continue
                if gui or self.godot:
                    next_deadline += time_step
                    pace_until(next_deadline)
                    if next_deadline < time.perf_counter() - time_step:
                        next_deadline = time.perf_counter()
                step += 1
            print(f"Strike timed out in {command.phase.value} phase")
            return finish(False, command.phase.value, max_seconds)
        except p.error:
            # The GUI can close between two PyBullet calls (for example while
            # the forward camera renders).  Preserve telemetry in that case;
            # other PyBullet failures remain visible to the caller.
            if not p.isConnected():
                return finish(False, command.phase.value, now_s, "PyBullet physics server closed")
            raise
        finally:
            if writer:
                writer.release()
            if plot_sender:
                send_plot_message(plot_sender, {"type": "close"})
                plot_sender.close()
            if plot_process:
                plot_process.join(timeout=1.0)
                if plot_process.is_alive():
                    plot_process.terminate()
            if self.godot:
                self.godot.close()

    @staticmethod
    def _video_writer(video: Path | None, config: StrikeConfig):
        if not video:
            return None
        video.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"mp4v"), config.camera_hz, config.environment_size_px)
        if not writer.isOpened():
            raise RuntimeError(f"Could not open video output: {video}")
        return writer

    def _print_summary(self, result: StrikeResult, summary: dict[str, object]) -> None:
        """Print one colored, human-readable summary for the completed run."""
        reset, cyan, green, red, yellow = "\033[0m", "\033[1;36m", "\033[1;32m", "\033[1;31m", "\033[1;33m"
        status = f"{green}target contacted{reset}" if result.success else f"{red}no valid contact{reset}"
        print(f"\n{cyan}--- TTC strike summary ---{reset}")
        print(f"drone: {yellow}{self.scene.drone_profile}{reset}; scenario: {yellow}{self.scenario_name}{reset}")
        print(f"result: {status}")
        print(f"final phase: {result.phase}; simulated time: {result.simulated_time_s:.1f} s")
        if result.phase == FlightPhase.ABORT.value and summary.get("abort_reason"):
            print(f"{red}ABORT: {summary['abort_reason']}{reset}")
        collision = summary["collision"]
        metrics = summary["flight_metrics"]
        print(f"starting pose: {summary['starting_pose']['position_m']}")
        print(f"target: center {summary['target']['center_m']}, size {summary['target']['size_m']:.2f} m")
        print(f"collision time: {collision['time_s']}")
        print(f"collision position: {collision['position_m']}")
        print(f"hitting velocity: {collision['velocity_mps']}")
        print(f"maximum altitude: {metrics['maximum_altitude_m']}")
        print(f"maximum forward speed: {metrics['maximum_forward_speed_mps']}")
        if result.impact_speed_mps:
            print(f"impact speed: {result.impact_speed_mps:.1f} m/s")
        if result.video:
            print(f"environment video: {result.video}")
        if result.plot:
            print(f"trajectory plot: {result.plot}")
        if result.csv:
            print(f"telemetry CSV: {result.csv}")
        if result.summary:
            print(f"run summary: {result.summary}")
        print("environment: red target cube and 3 static buildings")
