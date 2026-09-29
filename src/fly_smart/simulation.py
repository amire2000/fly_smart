"""PyBullet adapter that composes sensing, TTC, guidance, views, and telemetry."""

from dataclasses import dataclass
from math import sqrt
from pathlib import Path
import time

import cv2
import pybullet as p

from .common.drone_physics import PhysicsEngine, clamp
from .common.flight_control import AttitudeController
from .common.pybullet_sensors import read_imu
from .common.pybullet_utils import create_world, draw_force_vectors
from .forward_camera import add_environment_buildings, add_red_cube, forward_rgb
from .red_target_detector import detect_red_box

from .config import SceneConfig, StrikeConfig
from .guidance import FlightPhase, GuidanceCommand, GuidanceInput, StrikeGuidance
from .sensing import Barometer, BarometerReading, VerticalEstimator, VerticalImu
from .telemetry import FlightLog, build_summary, make_plot, move_plot_window, refresh_plot, save_csv, save_plot, save_summary
from .ttc import BboxTtcTracker, TtcObservation
from .views import annotate, environment_rgb

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

    def __init__(self, config: StrikeConfig | None = None, scene: SceneConfig | None = None) -> None:
        self.config = config or StrikeConfig()
        self.scene = scene or self.config.simulation

    def run(self, gui: bool, max_seconds: float, video: Path | None, plot: Path | None, csv: Path | None = None, summary: Path | None = None) -> StrikeResult:
        config = self.config
        model = self.scene.drone_model
        settings = self.scene.physics_settings
        physics_hz = settings.physics_hz
        time_step = settings.time_step_s
        control_steps = settings.control_steps
        drone = create_world(model, settings)
        engine = PhysicsEngine(model, settings)
        p.resetBasePositionAndOrientation(drone, config.launch_position, (0, 0, 0, 1))
        cube = add_red_cube(self.scene.target_center, self.scene.target_size_m)
        add_environment_buildings()
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
        force_lines = [-1, -1, -1, -1]
        renderer = p.ER_BULLET_HARDWARE_OPENGL if gui else p.ER_TINY_RENDERER
        impact_speed, stop_at_s = 0.0, None
        log = FlightLog()
        writer = self._video_writer(video, config)
        live_plot = self._live_plot(gui, plot, config, self.scene)
        if gui:
            cv2.namedWindow("TTC diagonal strike", cv2.WINDOW_NORMAL)
            cv2.moveWindow("TTC diagonal strike", *config.opencv_window_position_px)
            p.resetDebugVisualizerCamera(36.0, 48.0, -25.0, (7.0, 0.0, 7.0))

        def finish(success: bool, phase: str, now_s: float, abort_reason: str | None = None) -> StrikeResult:
            if plot:
                save_plot(log, config, self.scene, plot)
            if csv:
                save_csv(log, csv)
            if live_plot:
                refresh_plot(live_plot, log)
            result = StrikeResult(success, phase, now_s, impact_speed, video, plot, csv, summary)
            summary_data = build_summary(log, config, self.scene, success, phase, now_s, {"video": video, "plot": plot, "csv": csv, "summary": summary}, abort_reason)
            if summary:
                save_summary(summary_data, summary)
            self._print_summary(result, summary_data)
            return result

        def finish_if_disconnected(now_s: float) -> StrikeResult | None:
            """Save the partial run when closing the PyBullet GUI disconnects its server."""
            if p.isConnected():
                return None
            return finish(False, command.phase.value, now_s, "PyBullet physics server closed")

        try:
            for step in range(round(max_seconds / time_step)):
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
                            return finish(False, command.phase.value, now_s)
                        if command.phase == FlightPhase.ABORT:
                            # Guidance has already neutralized its pitch request;
                            # stop before applying another flight-control cycle.
                            last_height = tracker.last_observation.box[3] if tracker.last_observation else 0
                            return finish(
                                False,
                                command.phase.value,
                                now_s,
                                f"target lost before commit (last bbox height {last_height:g} px)",
                            )
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
                    log.append(now_s, position, velocity, command, pitch_rad, pitch_torque, observation, flight_step, baro)

                if stop_at_s is None and p.getContactPoints(drone, cube):
                    impact_speed = sqrt(sum(component**2 for component in incoming_velocity))
                    log.mark_collision(now_s, position, incoming_velocity)
                    stop_at_s = now_s + config.post_impact_seconds
                    print(f"Impact: {impact_speed:.1f} m/s; recording aftermath for {config.post_impact_seconds:.0f} s")
                    if live_plot:
                        refresh_plot(live_plot, log)
                elif stop_at_s is None and live_plot and step % (physics_hz // config.camera_hz) == 0:
                    refresh_plot(live_plot, log)
                if stop_at_s is not None and now_s >= stop_at_s:
                    # Contact is the geometry-free success condition.  Keep
                    # impact speed as telemetry instead of rejecting a valid
                    # strike because the simulated vehicle model is tuned
                    # differently from a real airframe.
                    return finish(True, "post-impact", now_s)

                if gui:
                    if frame is not None:
                        cv2.imshow("TTC diagonal strike", annotate(frame, command, observation))
                        if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                            return finish(False, command.phase.value, now_s)
                    draw_force_vectors(drone, flight_step, force_lines)
                    time.sleep(time_step)
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

    @staticmethod
    def _video_writer(video: Path | None, config: StrikeConfig):
        if not video:
            return None
        video.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"mp4v"), config.camera_hz, config.environment_size_px)
        if not writer.isOpened():
            raise RuntimeError(f"Could not open video output: {video}")
        return writer

    @staticmethod
    def _live_plot(gui: bool, output: Path | None, config: StrikeConfig, scene: SceneConfig):
        if not gui or not output:
            return None
        import matplotlib.pyplot as plt

        plt.ion()
        live_plot = make_plot(config, scene)
        live_plot.figure.canvas.manager.set_window_title("Live TTC strike telemetry")
        plt.show(block=False)
        move_plot_window(live_plot, config.plot_window_position_px)
        return live_plot

    @staticmethod
    def _print_summary(result: StrikeResult, summary: dict[str, object]) -> None:
        print("\n--- TTC strike summary ---")
        print(f"result: {'target contacted' if result.success else 'no valid contact'}")
        print(f"final phase: {result.phase}; simulated time: {result.simulated_time_s:.1f} s")
        if result.phase == FlightPhase.ABORT.value and summary.get("abort_reason"):
            print(f"\033[31mABORT: {summary['abort_reason']}\033[0m")
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
