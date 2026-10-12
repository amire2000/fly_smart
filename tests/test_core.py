"""Controller and vision checks through their production interfaces."""

from dataclasses import replace
from math import isclose, radians

import numpy as np
import pytest

from fly_smart.common.flight_control import VerticalSpeedController
from fly_smart.guidance import FlightPhase, GuidanceInput, StrikeGuidance, VerticalControlMode
from fly_smart.mission import MissionConfig
from fly_smart.red_target_detector import detect_red_box as legacy_detect_red_box
from fly_smart.trackers.red_target_detector import detect_red_box
from fly_smart.sensing import BarometerReading
from fly_smart.ttc import BboxTtcTracker, TtcObservation
from fly_smart.trajectory import TtcDescentPlanner, VerticalTargetPlanner
from fly_smart.visual_guidance import VisualEstimate, VisualGuidanceEstimator, world_bearing


def visual(t, tau=None, alpha=8.0, beta=0.0):
    observation = TtcObservation((300, 230, 20, 20), 20, 4, tau, 4, tau) if tau else None
    return VisualEstimate(t, round(t * 120), True, alpha, beta, observation=observation,
                          ttc_reliable=tau is not None, reason="test")


def tick(guidance, t, tau=None, alpha=8, beta=0, altitude=15, vz=0, missing=False):
    return guidance.update(GuidanceInput(t, BarometerReading(altitude, vz),
        None if missing else visual(t, tau, alpha, beta)))


def tracking(config=None):
    g = StrikeGuidance(config or MissionConfig())
    tick(g, 0)
    assert g.phase == FlightPhase.ACQUIRE
    for i in range(1, 20):
        tick(g, i / 120)
    assert g.phase == FlightPhase.TRACK
    return g


def test_core_guidance_and_vision_do_not_need_simulation():
    assert legacy_detect_red_box is detect_red_box
    config = MissionConfig()
    tracker = BboxTtcTracker(config)
    assert tracker.update((0, 0, 20, 20), 0) is None
    assert tracker.update((0, 0, 30, 30), .1).ttc_s > 0
    command = StrikeGuidance(config).update(GuidanceInput(0, BarometerReading(0, 0)))
    assert command.thrust_n > config.hover_thrust_n
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    frame[20:70, 20:70] = (255, 0, 0)
    assert detect_red_box(frame)[1] == (20, 20, 50, 50)
    frame[:] = (20, 89, 235)
    assert detect_red_box(frame)[1] is None


def test_dy_pd_deadband_damping_and_slew():
    planner = VerticalTargetPlanner(MissionConfig())
    planner.command(15, 8, 0, None, .1)
    assert planner.dy_mps == pytest.approx(-1.4)
    assert planner.velocity_mps == pytest.approx(-.1)
    planner.command(15, 8, 10, None, .1)
    assert planner.dy_mps == pytest.approx(-1.6)
    for alpha in (1, 0, -20):
        planner.command(15, alpha, 30, None, .1)
        assert planner.dy_mps == 0
    planner.command(15, 20, 0, None, .1)
    assert planner.dy_mps == -2
    assert TtcDescentPlanner(MissionConfig()).command(None, 15).vertical_velocity_mps == 0


def test_continuous_ttc_handoff_and_loss_preserves_pi():
    g = tracking()
    commands = [tick(g, i / 120, tau=8) for i in range(20, 140)]
    modes = {x.vertical_control_mode for x in commands}
    assert modes == {VerticalControlMode.DY, VerticalControlMode.BLEND, VerticalControlMode.TTC}
    assert commands[-1].ttc_weight == 1
    previous = commands[-1].vertical_velocity_target_mps
    old_integral = g.vertical_control.pid.integral
    command = tick(g, 140 / 120)
    assert command.vertical_control_mode == VerticalControlMode.DY
    assert command.ttc_weight == 0
    assert abs(command.vertical_velocity_target_mps - previous) <= 1 / 120 + 1e-9
    assert abs(g.vertical_control.pid.integral - old_integral) <= 5.5 / 120
    # Reliable TTC above entry threshold is retained within the exit threshold.
    for i in range(141, 250):
        tick(g, i / 120, tau=8)
    assert tick(g, 250 / 120, tau=9).vertical_control_mode == VerticalControlMode.TTC
    assert tick(g, 251 / 120, tau=11).vertical_control_mode == VerticalControlMode.DY


def test_bearing_loss_recovery_and_timeout():
    g = tracking()
    before = tick(g, .2)
    lost = tick(g, .25, missing=True)
    assert lost.phase == FlightPhase.RECOVER
    assert lost.pitch_target_rad == before.pitch_target_rad
    braking = tick(g, .45, missing=True)
    assert braking.pitch_target_rad < lost.pitch_target_rad
    for i in range(55, 70):
        command = tick(g, i / 120)  # reacquisition needs bearing, not TTC
    assert command.phase == FlightPhase.TRACK
    tick(g, .6, missing=True)
    assert tick(g, 1.61, missing=True).phase == FlightPhase.ABORT


def test_stale_frame_cannot_hold_guidance_or_arm_commit():
    g = tracking()
    stale = visual(.1, tau=.2, alpha=0)
    command = g.update(GuidanceInput(.4, BarometerReading(1, 0), stale))
    assert command.phase == FlightPhase.RECOVER


def test_blind_commit_is_capture_anchored_and_expires():
    g = tracking()
    eligible = tick(g, .2, tau=.25, alpha=0, altitude=1)
    committed = tick(g, .24, missing=True, altitude=1)
    assert committed.phase == FlightPhase.COMMIT
    assert committed.vertical_velocity_target_mps == eligible.vertical_velocity_target_mps
    assert g.commit_deadline_s == pytest.approx(.45)
    command = tick(g, .451, missing=True, altitude=1)
    assert command.phase == FlightPhase.ABORT and command.commit_expired


def test_late_ttc_and_stopping_distance_enter_recovery():
    g = tracking()
    assert tick(g, .2, tau=2).phase == FlightPhase.RECOVER
    g = tracking()
    assert tick(g, .2, altitude=1.2, vz=-3).reason == "descent_infeasible"


def test_terminal_braking_and_altitude_hold():
    p = VerticalTargetPlanner(MissionConfig())
    p.initialize(-2)
    before = p.velocity_mps
    p.command(1.05, 20, 0, .4, .1)
    assert p.requested_mps == pytest.approx(-.04)
    assert p.velocity_mps == pytest.approx(before + .1)
    p.command(.9, 20, 0, None, .1)
    assert p.requested_mps > 0
    assert not p.feasible(1.2, -3, None)
    assert p.feasible(15, 0, 8)
    assert not p.feasible(15, 0, 5)


def test_vertical_pi_antiwindup_unwind_and_tilt():
    c = MissionConfig()
    controller = VerticalSpeedController(c, c.hover_thrust_n * 2)
    for _ in range(100):
        controller.update(100, 0, 0, 0, .01)
    assert controller.pid.integral == 0
    assert controller.saturated
    controller.pid.integral = 1
    controller.update(0, 1, 0, 0, .1)
    assert controller.pid.integral == pytest.approx(.9)
    a = VerticalSpeedController(c, 30)
    b = VerticalSpeedController(c, 30)
    assert b.update(0, 0, radians(10), radians(10), .01) > a.update(0, 0, 0, 0, .01)


def test_calibrated_bearing_mount_attitude_and_lateral_sign():
    c = MissionConfig()
    box = (310, 230, 20, 20)
    assert world_bearing(box, c, (0, 0, 0), 0) == (0, 0)
    alpha, beta = world_bearing(box, c, (0, radians(10), radians(20)), radians(20))
    assert alpha == pytest.approx(10) and beta == pytest.approx(0)
    alpha, _ = world_bearing(box, replace(c, camera_look_down_deg=10), (0, 0, 0), 0)
    assert alpha == pytest.approx(10)
    assert world_bearing((350, 230, 20, 20), c, (0, 0, 0), 0)[1] < 0
    g = tracking()
    assert tick(g, .2, beta=radians(5)).roll_target_rad < 0


def test_unique_frames_and_independent_ttc_confidence():
    estimator = VisualGuidanceEstimator(MissionConfig())
    for i in range(20):
        estimate = estimator.update((310, 250, 20, 20), i / 30, i, (0, 0, 0), i / 30)
    assert estimate.bearing_valid and not estimate.ttc_reliable
    count = len(estimator.history)
    assert estimator.update((0, 0, 100, 100), 99, 19, (0, 0, 0), 99) is estimate
    assert len(estimator.history) == count
    estimator.reset()
    for i in range(20, 50):
        scale = 20 + (i - 20)
        estimate = estimator.update((300, 240, scale, scale), i / 30, i, (0, 0, 0), i / 30)
    assert estimate.bearing_valid and estimate.ttc_reliable
    assert not estimator.update((0, 0, 30, 30), 50 / 30, 50, (0, 0, 0), 50 / 30).ttc_reliable
    assert not estimator.update(None, 51 / 30, 51, (0, 0, 0), 51 / 30).bearing_valid


def test_invalid_sensor_and_acquire_timeout_are_finite():
    g = StrikeGuidance(MissionConfig())
    tick(g, 0, missing=True)
    assert tick(g, 5.1, missing=True).phase == FlightPhase.ABORT
    bad = g.update(GuidanceInput(5.2, BarometerReading(float("nan"), 0)))
    assert bad.phase == FlightPhase.ABORT and isclose(bad.thrust_n, MissionConfig().hover_thrust_n)


def test_ttc_cannot_relax_visible_bearing_descent_before_braking():
    p = VerticalTargetPlanner(MissionConfig())
    p.command(4, 8, 0, 7, .5)
    assert p.weight == 1
    assert p.requested_mps == pytest.approx(-1.4)
    p.command(4, 8, 0, 1, .1)
    assert p.requested_mps < -1.4


@pytest.mark.parametrize("tau,alpha,beta,altitude", [(None, 0, 0, 1), (.31, 0, 0, 1),
                                                  (.2, 3, 0, 1), (.2, 0, radians(3), 1), (.2, 0, 0, 15)])
def test_blind_commit_rejects_unqualified_terminal_state(tau, alpha, beta, altitude):
    g = tracking()
    tick(g, .2, tau=tau, alpha=alpha, beta=beta, altitude=altitude)
    command = tick(g, .24, missing=True, altitude=altitude)
    assert command.phase != FlightPhase.COMMIT


def test_reference_slew_uses_actual_elapsed_time():
    a = VerticalTargetPlanner(MissionConfig())
    b = VerticalTargetPlanner(MissionConfig())
    for _ in range(100):
        a.command(15, 20, 0, None, .01)
    for _ in range(10):
        b.command(15, 20, 0, None, .1)
    assert a.velocity_mps == pytest.approx(b.velocity_mps)


def test_association_jump_and_noisy_scale_reset_ttc_without_using_stale_frames():
    e = VisualGuidanceEstimator(MissionConfig())
    e.update((310, 230, 20, 20), 0, 0, (0, 0, 0), 0)
    jumped = e.update((590, 230, 20, 20), .03, 1, (0, 0, 0), .03)
    assert not jumped.bearing_valid and jumped.reason == "bearing_association_jump"
    e.reset()
    for i in range(2, 25):
        side = 40 if i % 2 else 20
        estimate = e.update((320 - side // 2, 240 - side // 2, side, side), i / 30, i, (0, 0, 0), i / 30)
    assert estimate.bearing_valid and not estimate.ttc_reliable


def test_feasibility_uses_climb_limits_after_altitude_undershoot():
    p = VerticalTargetPlanner(MissionConfig())
    assert not p.feasible(0, 0, .3)
    assert p.feasible(0, 0, 3)
    assert not p.feasible(15, -6, 10)
    assert p.feasible(.95, 0, .1)


def test_invalid_attitude_has_bounded_abort_output():
    g = StrikeGuidance(MissionConfig(), max_thrust_n=2.)
    command = g.update(GuidanceInput(0, BarometerReading(0, 0), attitude_rad=(0, 0)))
    assert command.phase == FlightPhase.ABORT and command.thrust_n <= 2.
    e = VisualGuidanceEstimator(MissionConfig())
    assert not e.update((310, 230, 20, 20), 0, 0, (0, 0), 0).bearing_valid


def test_hold_telemetry_reports_the_active_altitude_target():
    g = StrikeGuidance(MissionConfig())
    command = tick(g, 0)
    assert command.phase == FlightPhase.ACQUIRE
    assert command.trajectory.altitude_target_m == 15
    g = tracking()
    command = tick(g, .2, altitude=10, missing=True)
    assert command.phase == FlightPhase.RECOVER
    assert command.trajectory.altitude_target_m == 10


def test_recovery_grace_preserves_fast_ttc_reference():
    g = tracking()
    tick(g, .2, altitude=30, vz=-4)
    g.planner.initialize(-4)
    command = tick(g, .25, altitude=30, vz=-4, missing=True)
    assert command.phase == FlightPhase.RECOVER
    assert command.vertical_velocity_target_mps == -4
    braking = tick(g, .45, altitude=30, vz=-4, missing=True)
    assert braking.vertical_velocity_target_mps > -4
