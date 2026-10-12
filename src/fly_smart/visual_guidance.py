"""Fresh calibrated bearing and independently gated visual TTC estimation."""

from collections import deque
from dataclasses import dataclass
from math import atan2, cos, degrees, exp, hypot, isfinite, radians, sin, sqrt, tan

from .common.flight_control import wrap_angle
from .mission import MissionConfig
from .ttc import BboxTtcTracker, TtcObservation


@dataclass(frozen=True)
class VisualEstimate:
    """Capture-time bearings; confidence flags never imply fresh data forever."""

    captured_at_s: float
    frame_id: int
    bearing_valid: bool = False
    alpha_deg: float = 0.0
    beta_rad: float = 0.0
    alpha_rate_deg_s: float = 0.0
    beta_rate_rad_s: float = 0.0
    observation: TtcObservation | None = None
    ttc_reliable: bool = False
    reason: str = "target_missing"


def world_bearing(box, config: MissionConfig, attitude: tuple[float, float, float], heading_rad: float) -> tuple[float, float]:
    """Return positive-down elevation and azimuth relative to initial heading.

    Body axes are forward +X, left +Y, up +Z; image-right is body -Y.
    Positive Euler pitch rotates the nose downward.
    """
    x, y, w, h = box
    focal = config.camera_width_px / (2 * tan(radians(config.camera_fov_deg) / 2))
    ray = (1.0, -(x + w / 2 - config.camera_width_px / 2) / focal,
           -(y + h / 2 - config.camera_height_px / 2) / focal)
    mount = radians(config.camera_look_down_deg)
    bx = cos(mount) * ray[0] + sin(mount) * ray[2]
    by = ray[1]
    bz = -sin(mount) * ray[0] + cos(mount) * ray[2]
    roll, pitch, yaw = attitude
    by, bz = cos(roll) * by - sin(roll) * bz, sin(roll) * by + cos(roll) * bz
    bx, bz = cos(pitch) * bx + sin(pitch) * bz, -sin(pitch) * bx + cos(pitch) * bz
    wx, wy = cos(yaw) * bx - sin(yaw) * by, sin(yaw) * bx + cos(yaw) * by
    return degrees(atan2(-bz, hypot(wx, wy))), wrap_angle(atan2(wy, wx) - heading_rad)


class VisualGuidanceEstimator:
    """Own frame identity, bearing filtering, scale history and TTC eligibility."""

    def __init__(self, config: MissionConfig) -> None:
        self.config = config
        self.tracker = BboxTtcTracker(config)
        self.history = deque()
        self.estimate: VisualEstimate | None = None
        self.last_frame_id: int | None = None
        self.last_bearing_time: float | None = None
        self.heading_rad = 0.0

    def reset(self) -> None:
        """Discard takeoff growth and derivatives without changing frame identity."""
        self.tracker.reset()
        self.history.clear()
        self.last_bearing_time = None
        self.estimate = None

    def update(self, box: tuple[int, int, int, int] | None, captured_at_s: float,
               frame_id: int, attitude: tuple[float, float, float], now_s: float) -> VisualEstimate | None:
        """Consume a unique frame; duplicates leave both filters and age unchanged."""
        if frame_id == self.last_frame_id:
            return self.estimate
        self.last_frame_id = frame_id
        age = now_s - captured_at_s
        if len(attitude) != 3 or not all(isfinite(v) for v in (*attitude, captured_at_s, now_s)) or not 0 <= age <= self.config.frame_max_age_s:
            return self._invalid(captured_at_s, frame_id, "frame_stale_or_attitude_invalid")
        if box is None:
            return self._invalid(captured_at_s, frame_id, "target_missing")
        if len(box) != 4:
            return self._invalid(captured_at_s, frame_id, "bbox_invalid")
        x, y, w, h = box
        if not all(isfinite(v) for v in box):
            return self._invalid(captured_at_s, frame_id, "bbox_invalid")
        if w < 3 or h < 3 or x < 0 or y < 0 or x + w > self.config.camera_width_px or y + h > self.config.camera_height_px:
            return self._invalid(captured_at_s, frame_id, "bbox_invalid")
        alpha, beta = world_bearing(box, self.config, attitude, self.heading_rad)
        dt = captured_at_s - self.last_bearing_time if self.last_bearing_time is not None else None
        if dt is not None and dt <= 0:
            return self._invalid(captured_at_s, frame_id, "capture_time_not_increasing")
        old = self.estimate
        discontinuity = dt is None or dt > self.config.frame_max_age_s
        if old and old.bearing_valid and (abs(alpha - old.alpha_deg) > 20 or abs(wrap_angle(beta - old.beta_rad)) > radians(20)):
            discontinuity = True
            self.reset()
            return self._invalid(captured_at_s, frame_id, "bearing_association_jump")
        if discontinuity:
            self.tracker.reset()
            self.history.clear()
        arate = brate = 0.0
        if old and old.bearing_valid and not discontinuity:
            weight = 1 - exp(-dt / self.config.bearing_filter_time_s)
            filtered_alpha = old.alpha_deg + weight * (alpha - old.alpha_deg)
            filtered_beta = wrap_angle(old.beta_rad + weight * wrap_angle(beta - old.beta_rad))
            arate = old.alpha_rate_deg_s + weight * ((filtered_alpha - old.alpha_deg) / dt - old.alpha_rate_deg_s)
            brate = old.beta_rate_rad_s + weight * (wrap_angle(filtered_beta - old.beta_rad) / dt - old.beta_rate_rad_s)
            alpha, beta = filtered_alpha, filtered_beta
        self.last_bearing_time = captured_at_s
        clipped = x < 2 or y < 2 or x + w > self.config.camera_width_px - 2 or y + h > self.config.camera_height_px - 2
        observation = self.tracker.update(box, captured_at_s)
        if clipped:
            self.history.clear()
            self.tracker.reset()
        else:
            self.history.append((captured_at_s, sqrt(w * h)))
            while self.history and captured_at_s - self.history[0][0] > self.config.ttc_window_s + 1e-6:
                self.history.popleft()
        reliable, reason = self._ttc_gate(observation, clipped)
        self.estimate = VisualEstimate(captured_at_s, frame_id, True, alpha, beta, arate, brate, observation, reliable, reason)
        return self.estimate

    def _invalid(self, time_s: float, frame_id: int, reason: str) -> VisualEstimate:
        self.history.clear()
        self.tracker.reset()
        self.last_bearing_time = None
        self.estimate = VisualEstimate(time_s, frame_id, reason=reason)
        return self.estimate

    def _ttc_gate(self, observation: TtcObservation | None, clipped: bool) -> tuple[bool, str]:
        if clipped:
            return False, "scale_clipped"
        if len(self.history) < self.config.ttc_min_samples or self.history[-1][0] - self.history[0][0] < self.config.ttc_window_s * 0.8:
            return False, "scale_window_short"
        times, scales = zip(*self.history)
        mt, ms = sum(times) / len(times), sum(scales) / len(scales)
        variance = sum((t - mt) ** 2 for t in times)
        slope = sum((t - mt) * (s - ms) for t, s in self.history) / variance
        residual = sqrt(sum((s - (ms + slope * (t - mt))) ** 2 for t, s in self.history) / len(times)) / ms
        if slope <= 0 or slope * (times[-1] - times[0]) <= 1:
            return False, "scale_growth_unresolved"
        if residual > self.config.ttc_fit_max_relative_rms:
            return False, "scale_fit_noisy"
        if observation is None or not isfinite(observation.ttc_s) or observation.ttc_s <= 0:
            return False, "filtered_growth_invalid"
        return True, "ttc_qualified"
