"""Pure camera geometry helpers shared by guidance and telemetry."""

from math import atan, degrees, isfinite, radians, tan


def bbox_alignment_angles(
    box: tuple[int, int, int, int] | None,
    camera_width_px: float,
    camera_height_px: float,
    horizontal_fov_deg: float,
    measured_pitch_rad: float = 0.0,
) -> tuple[float, float, float]:
    """Return bbox dx, dy, and pitch-compensated dy angles in degrees."""
    if box is None:
        return float("nan"), float("nan"), float("nan")
    x, y, width, height = box
    dx_px = x + width / 2.0 - camera_width_px / 2.0
    dy_px = y + height / 2.0 - camera_height_px / 2.0
    horizontal_fov_rad = radians(horizontal_fov_deg)
    vertical_fov_rad = 2.0 * atan(tan(horizontal_fov_rad / 2.0) * camera_height_px / camera_width_px)
    dx_deg = degrees(atan((dx_px / (camera_width_px / 2.0)) * tan(horizontal_fov_rad / 2.0)))
    dy_deg = degrees(atan((dy_px / (camera_height_px / 2.0)) * tan(vertical_fov_rad / 2.0)))
    pitch_deg = degrees(measured_pitch_rad)
    # PyBullet's positive Y Euler pitch is nose-down for this vehicle model;
    # add it to the image-down angle to recover the world-relative error.
    compensated_dy_deg = dy_deg + pitch_deg if isfinite(pitch_deg) else float("nan")
    return dx_deg, dy_deg, compensated_dy_deg
