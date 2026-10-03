# DY-to-TTC plot marker

## Intent

Make the first vertical-control handoff from bbox-DY control to TTC control
visible in the telemetry plots. This is a display-only diagnostic; it does not
change guidance, telemetry sampling, or the 3 Hz live-plot refresh rate.

## Behavior

The plot scans the existing `vertical_control_mode` samples and marks the
first adjacent `dy` to `ttc` transition. A dashed `DY → TTC` line is drawn on
the time-based telemetry axes. The world `x-z` path plot is excluded because
its horizontal axis is distance rather than time. Later re-entries into TTC
are intentionally not marked.

## Validation

The telemetry tests verify the transition timestamp, the marker count, and the
first-transition-only behavior.
