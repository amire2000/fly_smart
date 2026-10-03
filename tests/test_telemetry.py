from math import atan, degrees, isclose, radians, tan

from fly_smart.guidance import FlightPhase, GuidanceCommand
from fly_smart.simulation.config import StrikeConfig
from fly_smart.simulation.telemetry import FlightLog, make_plot, refresh_plot
from fly_smart.trajectory import TrajectoryCommand
from fly_smart.ttc import TtcObservation


def test_telemetry_records_the_pid_vertical_target():
    command = GuidanceCommand(
        FlightPhase.TRACK,
        1.0,
        0.0,
        TrajectoryCommand(13.0, -1.5, 15.0),
        vertical_velocity_target_mps=-4.5,
    )
    log = FlightLog()
    log.append(0.0, (0.0, 0.0, 15.0), (0.0, 0.0, 0.0), command)
    assert log.command_vz_mps == [-1.5]
    assert log.pid_vz_target_mps == [-4.5]


def test_bbox_center_errors_are_recorded_in_pixels():
    command = GuidanceCommand(FlightPhase.TRACK, 1.0, 0.0, TrajectoryCommand(13.0, -1.5, 15.0))
    observation = TtcObservation((330, 250, 20, 20), 20.0, 2.0, 10.0, 2.0, 10.0)
    log = FlightLog()
    log.append(0.0, (0.0, 0.0, 15.0), (0.0, 0.0, 0.0), command, observation=observation)
    assert log.bbox_center_dx_px == [20.0]
    assert log.bbox_center_dy_px == [20.0]


def test_camera_angles_and_pitch_compensation_use_configured_geometry():
    command = GuidanceCommand(FlightPhase.TRACK, 1.0, 0.0, TrajectoryCommand(13.0, -1.5, 15.0))
    observation = TtcObservation((330, 250, 20, 20), 20.0, 2.0, 10.0, 2.0, 10.0)
    log = FlightLog()
    log.append(0.0, (0.0, 0.0, 15.0), (0.0, 0.0, 0.0), command, measured_pitch_rad=radians(5.0), observation=observation)
    expected_dx = degrees(atan((20.0 / 320.0) * tan(radians(90.0) / 2.0)))
    vertical_fov = 2.0 * atan(tan(radians(90.0) / 2.0) * 480.0 / 640.0)
    expected_dy = degrees(atan((20.0 / 240.0) * tan(vertical_fov / 2.0)))
    assert isclose(log.bbox_center_dx_deg[0], expected_dx)
    assert isclose(log.bbox_center_dy_deg[0], expected_dy)
    assert isclose(log.pitch_compensated_dy_deg[0], expected_dy + 5.0)


def test_camera_compensation_is_nan_without_a_detection():
    command = GuidanceCommand(FlightPhase.TRACK, 1.0, 0.0, TrajectoryCommand(13.0, -1.5, 15.0))
    log = FlightLog()
    log.append(0.0, (0.0, 0.0, 15.0), (0.0, 0.0, 0.0), command)
    assert log.bbox_center_dx_deg[0] != log.bbox_center_dx_deg[0]
    assert log.bbox_center_dy_deg[0] != log.bbox_center_dy_deg[0]
    assert log.pitch_compensated_dy_deg[0] != log.pitch_compensated_dy_deg[0]


def test_ttc_plot_shows_raw_filtered_seconds_and_activation_gate():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    config = StrikeConfig()
    plot = make_plot(config, config.simulation)
    log = FlightLog()
    command = GuidanceCommand(FlightPhase.TRACK, 1.0, 0.0, TrajectoryCommand(13.0, -1.5, 15.0))
    for time_s, ttc_s, raw_ttc_s in ((0.0, 11.0, 12.0), (0.1, 6.5, 7.0)):
        log.append(time_s, (time_s, 0.0, 15.0), (1.0, 0.0, 0.0), command, observation=TtcObservation((0, 0, 10, 10), 10.0, 1.0, raw_ttc_s, 1.0, ttc_s))
    refresh_plot(plot, log)
    assert plot.ttc_axis.get_ylabel() == "TTC (s)"
    assert list(plot.lines[17].get_ydata())[1] == 7.0
    assert list(plot.lines[18].get_ydata())[1] == 6.5
    assert list(plot.lines[17].get_ydata())[0] != list(plot.lines[17].get_ydata())[0]
    assert list(plot.lines[18].get_ydata())[0] != list(plot.lines[18].get_ydata())[0]
    assert list(plot.ttc_gate_line.get_xdata()) == [0.1, 0.1]
    assert plot.alignment_axis.get_ylabel() == "center error (px)"
    assert list(plot.lines[19].get_ydata()) == [-315.0, -315.0]
    assert list(plot.lines[20].get_ydata()) == [-235.0, -235.0]
    assert list(plot.lines[23].get_ydata()) == list(log.pitch_compensated_dy_deg)
    plt.close(plot.figure)


def test_plot_masks_late_growth_spikes_but_keeps_gate_markers():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    config = StrikeConfig()
    plot = make_plot(config, config.simulation)
    log = FlightLog()
    command = GuidanceCommand(FlightPhase.TRACK, 1.0, 0.0, TrajectoryCommand(13.0, -1.5, 15.0))
    for time_s, growth in ((0.0, 2.0), (0.1, 3.0), (0.2, 10000.0)):
        log.append(time_s, (time_s, 0.0, 15.0), (1.0, 0.0, 0.0), command, observation=TtcObservation((0, 0, 10, 10), 10.0, growth, 5.0, growth, 5.0))
    refresh_plot(plot, log)
    assert list(plot.growth_gate_line.get_xdata()) == [0.0, 0.0]
    assert list(plot.lines[16].get_ydata())[-1] != 10000.0
    plt.close(plot.figure)


def test_live_plot_can_render_bounded_recent_history():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    config = StrikeConfig()
    plot = make_plot(config, config.simulation)
    log = FlightLog()
    command = GuidanceCommand(FlightPhase.TRACK, 1.0, 0.0, TrajectoryCommand(13.0, -1.5, 15.0))
    for time_s in range(5):
        log.append(float(time_s), (float(time_s), 0.0, 15.0), (1.0, 0.0, 0.0), command)
    refresh_plot(plot, log, max_points=2)
    assert len(plot.lines[0].get_xdata()) == 2
    assert list(plot.lines[0].get_xdata()) == [3.0, 4.0]
    plt.close(plot.figure)
