# Godot interactive attempt controls

`--interactive` starts paused with `▶` and `↻` buttons in Godot's lower-left
toolbar. Godot sends the button action to Python over UDP; Start advances
PyBullet, while Reset pauses, restores all Python flight state and the received
Godot poses, clears the collision latch, and waits for Start again. The OpenCV
FPV overlay remains visible while paused.

Each reset begins a new numbered attempt folder below the run directory. The
attempt owns its video, CSV, plot, and summary. Python remains responsible for
flight movement; Godot receives a reset flag alongside the initial pose so its
display and collision trigger return to their initial state.

Validation: unit-test control state changes and bridge reset payloads; manually
confirm that Start waits, Restart resets both windows, and attempts have
distinct output folders.
