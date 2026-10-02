# Fly Smart simulation main loop

`StrikeSimulation.run()` is the orchestration loop for one or more TTC strike
attempts. It keeps the reusable flight core independent from the simulation
adapters: PyBullet advances the vehicle, the core estimates and guides flight,
and optional Godot/plot processes provide rendering and observation sinks.

## Responsibilities

| Module | Responsibility |
| --- | --- |
| `simulation.runner.StrikeSimulation` | Own initialization, clocks, attempt lifecycle, and terminal handling. |
| PyBullet and `PhysicsEngine` | Advance the drone and scene one physics timestep. |
| `simulation.sensors` and `VerticalEstimator` | Produce filtered altitude and vertical velocity from synthetic IMU/barometer data. |
| Camera and `detect_red_box` | Produce an RGB frame and red-target bounding box. |
| `BboxTtcTracker` | Convert bbox scale growth into raw and filtered TTC observations. |
| `StrikeGuidance` | Select the flight phase and high-level trajectory/thrust/pitch command. |
| `AttitudeController` | Convert desired pitch into body torque. |
| Godot bridge | Exchange poses, overlays, frames, collision events, and operator controls. |
| Telemetry and plot process | Record artifacts and optionally render live UDP-fed plots. |

The reusable core (`guidance`, `trajectory`, `sensing`, `ttc`, and target
tracking) does not call PyBullet or Godot. The runner supplies observations and
consumes commands at the simulation seam.

## Clock domains

The default drone profile advances the loop at 240 Hz and updates guidance at
120 Hz. The camera/detector runs at 30 Hz, so the latest detection is reused
between camera samples. The optional plot process refreshes at 3 Hz and is not
on the physics critical path.

One iteration advances simulated time by `1 / physics_hz`. Real-time pacing is
enabled for GUI or Godot runs. The reported real-time factor is:

```text
RTF = simulated elapsed seconds / wall-clock running seconds
```

## Control flow

```mermaid
flowchart TD
    A[CLI loads StrikeConfig] --> B[Create PyBullet world and scene]
    B --> C[Open optional Godot and plot process]
    C --> D[Reset attempt state]
    D --> E{Interactive controls?}
    E -->|Waiting| F[Poll Start Reset Stop]
    F -->|Reset| D
    F -->|Start| G[Begin attempt clock]
    F -->|Stop or close| Z[Save failure and clean up]
    E -->|Headless| G
    G --> H{Physics step remains?}
    H -->|No| Y[Save timeout result]
    H -->|Yes| I[Poll controls and connection]
    I --> J[Read PyBullet pose and velocity]
    J --> K[Sample IMU and barometer]
    K --> L[Update vertical estimator]
    L --> M{Camera tick?}
    M -->|No| N[Reuse latest detection state]
    M -->|Yes, headless| O[Render PyBullet camera]
    M -->|Yes, Godot| P[Read Godot shared-memory RGB frame]
    O --> Q[Detect red bbox and update TTC tracker]
    P --> Q
    N --> R{Control tick?}
    Q --> R
    R -->|No| U[Reuse latest command and torque]
    R -->|Yes| S[StrikeGuidance phase update]
    S --> T[AttitudeController torque update]
    T --> U
    U --> V[PhysicsEngine maps command and advances PyBullet]
    V --> W[Record telemetry and publish overlays/plot sample]
    W --> X{Collision or terminal event?}
    X -->|Target collision| X1[Freeze control and record aftermath]
    X1 --> X2{Post-impact period complete?}
    X2 -->|No| H
    X2 -->|Yes| X3[Save successful result]
    X -->|Obstacle, abort, expiry, disconnect| Z
    X -->|None| AA[Optional real-time pacing]
    AA --> H
    X3 --> AB[Close writers, plot, Godot]
    Y --> AB
    Z --> AB
```

## Normal tick and event flow

The runner is the only component that coordinates the full cycle. Godot is
optional: without it, the runner renders a PyBullet camera directly and checks
PyBullet contacts; with it, Godot supplies the camera frame and collision
events while PyBullet remains the physics authority.

```mermaid
sequenceDiagram
    participant R as StrikeSimulation
    participant P as PyBullet/PhysicsEngine
    participant S as Sensors/Estimator
    participant C as Camera
    participant D as Detector/TTC tracker
    participant G as StrikeGuidance
    participant A as AttitudeController
    participant GD as Godot bridge
    participant T as Telemetry/Plot

    R->>P: read pose and velocity
    R->>S: sample IMU and barometer
    S-->>R: filtered altitude and vertical velocity
    alt Camera tick
        alt Godot mode
            R->>GD: read shared-memory RGB frame
            GD-->>R: RGB frame or no frame
        else Headless mode
            R->>C: render forward RGB frame
            C-->>R: RGB frame
        end
        R->>D: detect bbox and update(now, bbox)
        D-->>R: target visibility, alignment, TTC observation
    end
    alt Control tick
        R->>G: update(GuidanceInput)
        G-->>R: phase, trajectory, thrust, pitch target
        R->>A: update measured attitude and pitch target
        A-->>R: body torque
    end
    R->>P: map thrust and apply torque
    P-->>R: physics step and force telemetry
    R->>T: append telemetry and publish live sample
    opt Godot mode
        R->>GD: publish pose and overlay
        GD-->>R: collision/control events
    end
    alt Target collision
        R->>T: mark collision and send collision marker
        R->>P: zero active control during aftermath
    else Obstacle, abort, expiry, or stop
        R->>T: save unsuccessful attempt
    else No terminal event
        R->>R: pace deadline and increment physics step
    end
```

## Flight phases and termination

- **TAKEOFF** holds zero pitch while the altitude controller climbs and waits
  for a stable barometer estimate. The first fresh tracking observation resets
  the TTC tracker before entering `TRACK`.
- **TRACK** uses live detections. Invalid or long TTC keeps the guidance in its
  pre-TTC descent mode; a valid TTC enables terminal trajectory control.
- **COMMIT** begins after target loss only when the tracker has a commit-ready
  observation. It holds the last valid pitch/descent target until contact or
  the commit deadline.
- **ABORT** stops forward guidance when the target is lost before commit or a
  terminal condition cannot be trusted.
- A target collision records impact speed and runs passive post-impact
  recording. Obstacle collision, operator Stop, timeout, disconnect, and
  commit expiry save unsuccessful artifacts. Interactive Reset starts a fresh
  attempt without recreating the process.

## Data ownership

The runner owns mutable attempt state. Core modules receive immutable input
records and return command/observation records. Telemetry is appended after
each physics step, while the plot process receives only the newest serialized
sample over UDP so plotting cannot block physics. Godot receives poses and
overlays over UDP, reads camera pixels through shared memory, and sends control
and collision events over separate UDP sockets.
