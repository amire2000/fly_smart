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
- Measured pitch is positive nose-up. The compensated vertical error is
  `dy_angle_deg - measured_pitch_deg`, so a target at the world horizon
  remains near zero while the vehicle pitches nose-up.

The compensated signal is an angular line-of-sight/altitude proxy. Absolute
altitude in metres requires a calibrated target size or range, which the
current bbox observation does not provide.

## Validation

Tests cover centered/off-center pixel conversion, pitch compensation sign,
and missing observations or physics samples.
