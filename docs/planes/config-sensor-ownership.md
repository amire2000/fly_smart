# Simulation sensor configuration ownership

Raw synthetic sensor settings are simulator inputs, so the canonical YAML path
is now `simulation.sensors`:

- `barometer`: sample rate, noise, bias, drift, and smoothing weights.
- `imu`: sample rate and accelerometer noise/bias parameters.

Estimator tuning remains runtime-owned at `runtime.vertical_estimator` because
it changes how measurements are fused rather than how a sensor is simulated.

This is an intentional schema move. `runtime.sensors` is rejected with a
message directing users to `simulation.sensors`; existing scenario files must
be migrated. Python callers keep the existing flat `StrikeConfig` sensor
attributes through its grouped-config compatibility lookup.
