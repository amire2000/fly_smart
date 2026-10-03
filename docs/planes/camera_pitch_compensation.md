# Camera pitch compensation telemetry

## Intent

Record camera alignment in angular units and remove the vehicle pitch
component from the vertical line-of-sight error for tuning. This release is
telemetry-only; guidance and actuator commands are unchanged.

## Conventions and calculation

- The configured camera is authoritative for telemetry geometry.
- Bbox `dx` is positive to image-right and `dy` is positive downward.
- Pixel offsets are converted with the configured horizontal FOV. The
  vertical FOV is derived from the configured aspect ratio.
- PyBullet's measured Y pitch is positive nose-down for this vehicle. The
  compensated vertical error is `dy_angle_deg + measured_pitch_deg`, so the
  vehicle attitude is removed in the simulator's actual sign convention.

The compensated signal is an angular line-of-sight/altitude proxy. Absolute
altitude in metres requires a calibrated target size or range, which the
current bbox observation does not provide.

## Vertical-control use

During tracking, the controller has two explicit modes. With a visible target
and no usable TTC (no observation or `ttc_s > ttc_activation_s`), it keeps the
existing no-TTC descent and adds a bounded camera correction:

`dy_correction_mps = clamp(-gain * deadbanded_compensated_dy_deg, +/- limit)`.

The correction is applied before the existing altitude-position correction and
vertical-velocity PID. At or below the 8 s activation gate, the TTC trajectory
is used and the camera correction is zero. Target loss keeps the existing
COMMIT/ABORT behavior.

Initial tuning is deliberately conservative: 0.20 m/s/deg gain, 1 degree
deadband, and a 2.0 m/s correction limit.

## Validation

Tests cover centered/off-center pixel conversion, pitch compensation sign,
DY/TTC mode selection, correction direction and saturation, and missing target
behavior. The 30 m, 60 m, and 80 m scenarios are the acceptance runs.
