# Godot target-distance selector

## Decision

The Godot reset controls expose a dropdown with target distances of 30, 40,
50, 60, 70, and 80 metres. The default is 30 m. The dropdown is disabled
while the simulation is running and is applied only when Reset is pressed.

Godot sends the selected value as `target_distance_m` in its existing UDP
control message. Python validates the value and remains authoritative for the
target pose. Distance is measured along positive world X from the configured
launch position; the configured target Y, Z, and 2 m size are preserved.

The distance control is shown in a small reset popup instead of occupying the
toolbar during flight. Confirming the popup sends the reset command. On reset,
Python moves the target reference, resets the attempt state, and publishes the
active distance back to Godot so the selector stays in sync.

Interactive terminal outcomes do not tear down the Python session. The runner
pauses after success, timeout, or an abort and continues polling Godot; a new
reset starts another attempt. Closing the interactive session remains the
explicit exit path.

## Validation

The supported values are an explicit finite set in Python. Invalid or missing
values leave the current target unchanged. Tests cover control-message
decoding and the preset-to-position calculation.
