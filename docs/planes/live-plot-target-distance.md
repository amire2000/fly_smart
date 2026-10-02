# Live plot target distance updates

The plotting subprocess receives a `target` message whenever an interactive
attempt is reset. It updates the existing scene-target marker to the active
target X/Z coordinates, so later distance selections do not remain fixed at
the initial 30 m target. Reset still clears telemetry independently.
