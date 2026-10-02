# Godot interactive controls

Move Start/Reset ownership from the Matplotlib telemetry figure into the
Godot overlay. Godot shows compact `▶` and `↻` buttons in the lower-left
corner and sends validated localhost UDP commands to Python on port 9102.

```mermaid
sequenceDiagram
    participant G as Godot buttons
    participant C as UDP 127.0.0.1:9102
    participant P as GodotBridge
    participant S as PyBullet runner
    participant R as UDP pose 9100

    G->>C: {event: simulation_control, action: start}
    C->>P: control event
    P->>S: running = true
    S->>R: drone and target poses

    G->>C: {event: simulation_control, action: reset}
    C->>P: control event
    P->>S: reset physics and controller state
    S->>R: reset=true plus initial poses
    R->>G: restore poses and collision latch
```

The `--interactive` runner waits for `start`, pauses on `reset`, and keeps the
telemetry figure display-only. Godot does not own simulation state; it only
emits UI commands and receives reset poses.

Validation: test control-message parsing and state transitions, load the Godot
scene headlessly, then manually confirm Start advances and Reset restores both
rendered poses and Python telemetry.
