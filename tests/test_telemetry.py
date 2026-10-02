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
    assert list(plot.lines[14].get_ydata()) == [12.0, 7.0]
    assert list(plot.lines[15].get_ydata()) == [11.0, 6.5]
    assert plot.ttc_gate_line.get_ydata()[0] == config.ttc_activation_s
    assert plot.alignment_axis.get_ylabel() == "center error (px)"
    assert list(plot.lines[16].get_ydata()) == [-315.0, -315.0]
    assert list(plot.lines[17].get_ydata()) == [-235.0, -235.0]
    plt.close(plot.figure)
