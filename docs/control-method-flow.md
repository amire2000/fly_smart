# Image-to-control method flow

This diagram shows one control cycle from an incoming camera image to motor
outputs. The camera runs at its configured frame rate; guidance and physics run
at their own control rates using the latest available visual observation.

```mermaid
flowchart TD
    image[RGB camera image] --> red[detect_red_box\nHSV red mask + largest contour]
    red --> visible{Bounding box found?}
    visible -- no --> lost[Target not visible]
    lost --> decision{Commit armed and\nlast TTC valid?}
    decision -- yes --> commit[COMMIT\nHold last pitch and descent target\nuntil TTC deadline]
    decision -- no --> abort[ABORT\nPitch = 0\nDamp vertical velocity]

    visible -- yes --> bbox[Bbox x, y, width, height]
    bbox --> scale["scale = sqrt(width × height)"]
    scale --> filter[Alpha-beta scale/growth filter]
    filter --> valid{Filtered growth >\nmin_growth_px_per_s?}

    valid -- yes --> ttc[TTC = filtered scale / filtered growth]
    ttc --> trajectory[TTC trajectory\nvz = impact-altitude error / TTC]
    valid -- no --> fallback[TTC unavailable\nUse configured feed-forward descent\nAdd optional no-growth pitch boost]
    fallback --> trajectory

    barometer[Barometer + vertical estimator\naltitude and vertical velocity] --> trajectory
    trajectory --> forward[Forward-speed PID\nforward-speed error to pitch target]
    trajectory --> vertical[Vertical correction + velocity PID\nvz error to collective thrust]
    forward --> pitch[Pitch target]
    vertical --> thrust[Collective thrust\ncompensated for measured pitch]

    imu[IMU\npitch and pitch rate] --> attitude[Pitch attitude PID]
    pitch --> attitude
    attitude --> torque[Pitch torque]
    torque --> mixer[Motor mixer]
    thrust --> mixer
    mixer --> motors[Four motor PWM outputs]
    motors --> vehicle[Drone motion]
    vehicle --> image
    vehicle --> barometer
    vehicle --> imu
```

## Decision rules

| Observation condition | Pitch and altitude action |
| --- | --- |
| Visible bbox and valid TTC growth | Track configured forward speed; descend to impact altitude over the estimated TTC. |
| Visible bbox but flat/shrinking/invalid TTC growth | Keep tracking; use the TTC-unavailable descent feed-forward and optional pitch boost. Do not fabricate a TTC. |
| Bbox absent after commit is armed with a valid final TTC | Hold the last valid pitch and corrected descent target until the commit deadline. |
| Bbox absent without a safe final TTC | Remove forward pitch and damp vertical velocity in abort. |

The motor mixer combines collective thrust with the pitch-torque correction.
The simulator applies those forces to the vehicle; a field adapter would send
the equivalent motor commands to the flight controller.

## TTC estimator flow

`BboxTtcTracker` runs only when a camera frame is available. This diagram
follows the exact decision path used to produce a `TtcObservation`.

```mermaid
flowchart TD
    frame[Camera frame] --> box{Red bbox detected?}
    box -- no --> no_target[Return no observation\nGuidance sees target_visible = false]
    box -- yes --> scale["scale = sqrt(width × height)"]
    scale --> commit[Update commit_ready when\nbbox height reaches commit threshold]
    commit --> initialized{Previous scale and time exist?}
    initialized -- no --> seed[Store first scale and timestamp\nReturn no TTC yet]
    initialized -- yes --> dt{dt > 0?}
    dt -- no --> invalid_time[Return no observation]
    dt -- yes --> predict[predicted scale = estimated scale\n+ estimated growth * dt]
    predict --> residual[residual = measured scale - predicted scale]
    residual --> raw[raw growth = measured-scale delta / dt\nraw TTC = scale / raw growth when raw growth > 0]
    raw --> update[estimated scale = predicted scale + alpha * residual\nestimated growth = previous growth + beta * residual / dt]
    update --> growth{estimated growth >\nmin_growth_px_per_s?}
    growth -- no --> unavailable[Return no valid TTC\nTarget can still be visible]
    growth -- yes --> observation[Return TtcObservation\nfiltered TTC = estimated scale / estimated growth]
    observation --> guidance[TRACK guidance\nTTC descent trajectory]
    unavailable --> fallback[TRACK guidance fallback\nfeed-forward descent and optional pitch boost]
```

The tracker retains the most recent valid observation separately. It is used
only for the bounded `COMMIT` path after the target leaves the camera view; a
flat or shrinking bbox never creates a synthetic TTC.
