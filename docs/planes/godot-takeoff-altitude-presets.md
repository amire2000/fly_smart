# Godot reset takeoff-altitude presets

Implementation commit: `pending`.

## Intent and interface

Add a takeoff-altitude selector with 20, 30 and 50 m to the existing Godot
reset popup beside target distance. Interpret altitude as the mission's takeoff
and acquisition height, not target-centre altitude or instantaneous teleportation.
Preserve the initial configured height (normally 15 m) as a selectable current
value, and allow returning to it. Python remains control authority.

Send `takeoff_altitude_m` with the existing reset event. Accept the explicit
presets or the initial configured altitude; reject missing, nonnumeric,
nonfinite or unsupported selections independently from distance. Stage selections
until reset: save the previous attempt against its previous settings, then
replace runtime takeoff altitude and rebuild controllers/estimators. Reset
returns to the launch pose, paused; Start climbs to the selected altitude.
Return actual applied altitude in the existing `target_control` payload and
record it in attempt summaries. Distance reset placement remains independent.
No mission gains, TTC gates, descent limits or target height change.

## Validation and limitations

Check UDP decoding, actual runner resets for every altitude and invalid input,
previous-attempt summary settings and Godot selector/packet synchronization.
Run pytest, component self-check and the isolated Godot scene harness.
This exposes configuration, not qualification of slant approaches from 50 m.
At short distances a higher takeoff can put the target outside the body-mounted
camera FOV and fail acquisition; pitch/speed/descent tuning is unchanged.

## Results

72 pytest tests and component self-check passed. The isolated Godot harness
verified the configured 15 m selection plus 20/30/50 m reset packets alongside
150 m distance, and retained bbox projection checks. Python reset integration
checks each valid selection, missing/invalid/NaN/infinite input, return to the
initial height, rebuilt guidance configuration, and old/new attempt summaries.
`git diff --check` passed. No complete approach qualification from these higher
starting altitudes was run.
