# DY-first descent, gated TTC and shared PI

Status: implemented and validated in simulation. Implementation commit:
`pending`. This supersedes altitude-hold-before-TTC and unconditional pre-TTC
slope descent. [Control explanation](../trajectory_algo.md),
[stages](../trajectory_stages.svg), [input graph](../trajectory_control_inputs.svg).

## Intent and sensor contract

Descend smoothly from reliable vertical bearing without waiting for TTC. Use
qualified TTC to refine descent timing, then brake toward a level finish at the
known target-centre altitude. Accuracy takes priority over contact speed.
Camera, IMU and barometer only: no simulator forward velocity, target distance,
target size or coordinates enter guidance. Simulation truth supplies synthetic
sensors and evaluates outcomes. One static red target is assumed.

Each camera observation carries a unique frame ID, capture time and matching
IMU attitude. Convert bbox centre through horizontal-FOV intrinsics, mounting
rotation and full roll/pitch/yaw into a world ray. Positive-down elevation
`alpha=atan2(-b.z,hypot(b.x,b.y))`; horizontal `beta` is azimuth relative to
initial yaw. Raw image dx/dy are not world-frame control errors.

## Control design

- Filter bearing and its rate on unique camera captures, time constant 0.15 s.
  Duplicates do not advance filters or refresh observation age.
- DY PD: `clamp(-0.20*max(alpha_deg-1,0)-0.02*alpha_rate_deg_s,-2,0)` m/s.
  Inside the 1-degree deadband return exactly zero. Integral disabled.
- TTC candidate: `(target_altitude-z)/max(TTC,0.2)`, bounded [-5.5,+3] m/s.
  Guard it with `min(DY,candidate)` so TTC cannot ease a useful DY descent too
  early. Blend weight ramps 0->1 in 0.5 s; unreliable TTC sets weight zero.
- Apply shared braking/speed bounds and a 1 m/s² reference slew after blending;
  seed reference from measured vertical speed after takeoff. At or below
  target altitude +0.2 m, use bounded altitude hold, including upward
  correction after an undershoot.
- Shared vertical-speed PI `(1.0,0.4,0)`, integral limit 5, actual dt. Preserve
  integral across DY/BLEND/TTC/RECOVER. Seed it from preceding thrust for a
  bumpless takeoff handoff. Conditional integration freezes only when error
  would worsen collective saturation; allow unwinding.
- Collective: `(mass*g+PI(vz_ref-vz))/(max(cos(roll)*cos(pitch),0.5))`, bounded
  to the adapter's declared actuator maximum. This is collective saturation,
  not measured motor allocation or battery-limit feedback.
- Horizontal bearing PD `(0.8,0,4.0)` drives roll, bounded +/-15 degrees and
  15 deg/s slew; hold initial yaw. Forward pitch is 7 degrees, slewed 5 deg/s; see the speed follow-up below.
  It is an approach request, not a measured speed target.
- Existing attitude PIDs produce body torques using actual dt. Roll tune
  `(0.02,0,0.01)`; pitch `(0.008,0,0.006)`. Legacy speed/slope/pitch-boost and
  bbox-size commit settings remain readable but are inactive in guidance.

## Confidence and states

Bearing requires valid centre/association/calibration/attitude and capture age
<=0.1 s. Reject abrupt bearing association changes >20 degrees. TTC adds eight
unique scale measurements in a 0.5 s window spanning >=0.4 s, >1 px fitted
growth, positive filtered growth, <=15% relative fit RMS and an unclipped box
(2 px border). Alpha-beta TTC uses alpha 0.85, beta 0.20. Reset scale history
on tracking discontinuity. Eligibility checks are not probabilities of success.

| State | Action and exit |
|---|---|
| TAKEOFF | Existing altitude PID/climb guard; settled altitude/speed -> ACQUIRE and discard takeoff TTC. |
| ACQUIRE | Hold takeoff altitude; fresh bearing for 0.1 s -> TRACK/DY; abort after 5 s. |
| TRACK/DY | Bearing descent; qualified TTC <=8 s for 0.3 s -> BLEND. |
| TRACK/BLEND | Ramp TTC weight for 0.5 s -> TTC; failure -> DY. |
| TRACK/TTC | Guarded TTC reference; confidence failure or TTC >10 s -> DY. |
| RECOVER | Bearing loss holds the preceding bounded command only until 0.2 s after last usable capture, then slews toward captured-altitude hold and zero pitch. Fresh bearing for 0.1 s and feasible descent -> DY. Loss/infeasibility reaching 1 s -> ABORT. |
| COMMIT | After target loss only: retain one coherent accepted RPY/vz command with fresh qualified TTC <=0.3 s, both bearings within 2 degrees and feasible motion. Deadline is capture time plus TTC, capped at 0.3 s; never extend it with reused frames. |
| ABORT | Finite hold/braking intent; simulation ends as failure. It is not a hardware landing procedure. |

TTC loss alone never enters RECOVER. Contact in any phase ends the attempt
successfully; motors stop for the aftermath. Invalid essential sensors/time
abort. PyBullet post-takeoff ground contact is reported separately from timeout.

## Braking and feasibility

For positive remaining height h, limit downward reference to
`sqrt((a*delay)^2+2*a*h)-a*delay`, with a=1 m/s² and delay=0.2 s. Check measured
stopping distance `v_down²/(2*a)+v_down*delay` against h plus the altitude
corridor. Compare acceleration/cruise/braking minimum duration from measured
vertical speed, ending at zero vertical speed, plus the reserve against
qualified TTC. Already stable inside the altitude corridor permits a level
finish. Infeasible motion enters recovery rather than raising descent limits.

This screening model is not a guarantee of physical acceleration/braking.
Bearing alone does not certify time remaining; reduced pitch cannot guarantee
horizontal braking. Physical overshoot, barometer bias, growing target image,
actuator limits and tracking losses remain relevant risks.

## SOLID implementation and adapters

Composition with small value interfaces, no class per state or abstract hierarchy:
visual estimation owns geometry/filter/history/confidence; trajectory planning
owns PD/blend/braking/feasibility; guidance owns states/timers/RPY; flight control
owns shared PI and attitude PIDs. Core has no simulation, PyBullet or Godot imports.
RuntimeConfig inherits core mission defaults so tunings cannot silently diverge.

The guidance input is now `GuidanceInput(now_s, barometer, visual, attitude_rad)`;
old raw observation/commit/true-forward-speed fields are removed. Visual output
is `VisualEstimate` with capture time, bearings/rates, TTC and eligibility.
`GuidanceCommand` adds roll/yaw targets, TTC weight, frame age, rejection reasons,
requested/limited vertical speed and collective-saturation diagnostics.
AttitudeController accepts explicit dt and optional roll tuning while retaining
its existing defaults for other callers. Compatibility import modules remain.

PyBullet now derives vertical FOV from configured horizontal FOV, supports
visibility to 500 m and uses a consistent rigid camera mount. Godot receives
runtime width/height/FOV/mount/rate, tags frames with rendered pose ID, and Python
matches capture-time IMU attitude. Duplicate poses do not count as new sensor
samples. Resizing atomically replaces the shared file and Python remaps it.
V-sync is disabled and render FPS bounded to avoid slow display refresh starving
the camera freshness gate. Confidence limits were not relaxed.

## Tuning review and validation

Initial 50/75/100 m baseline smoke contacted centrally. The first disturbance
matrix exposed lateral oscillation with outer PD (1,0,0.5) and the old weak roll
PID. Stronger outer derivative damping and faster roll feedback resolved the
+/-5 m offset and lateral-wind smoke cases. Tailwind exposed premature TTC easing;
growth-filter beta 0.20 improved response but did not alone fix the vertical error.
The bearing guard resolved the tailwind smoke, with final z approximately 1.02 m.
These changes were recorded before the related controller edits.

Godot initially supplied too-slow frames / invalid JSON from an absent speed
represented as NaN. Use explicit absence and strict JSON, disable display V-sync,
and reject stale/repeated captures. A normal-thread Godot smoke contacted at
approximately y=0, z=1.03 m. Earlier failed attempts remain under /tmp for audit.

Required checks: pure geometry/signs, association/quantization/noisy scale,
independent gates, handoff/no integral reset, actual dt, anti-windup, floor/braking,
late TTC, acquisition/recovery, strict commit and expiry, contact in every phase.
Run repository pytest/self-check plus headless and Godot smoke.

Full matrix: 50/75/100 m x seeds 0..9 x eight separate cases: baseline, target
lateral +/-5 m, wind +/-2 m/s by horizontal axis, and camera dropout at [9,9.2) s.
Keep default sensor noise, 2 m target and default vehicle. Score drone centre at
first target contact: abs(y-target_y)<=0.5 m and abs(z-target_z)<=0.5 m. This is a
contact-centre proxy, not a reconstructed physical surface-contact point.
Acceptance: all 30 baseline cases and >=95% of 210 disturbance cases centrally.
No combined-disturbance, multiple-target or hardware qualification claim.

```bash
uv run pytest -q
uv run fly-smart --self-check
uv run fly-smart --headless --config configs/scenario.yaml
# Start Godot first, then run the Godot camera path.
godot --path godot
uv run fly-smart --godot --headless --config configs/scenario.yaml
uv run python scripts/validate_trajectory.py --seeds 10 --workers 12 \
  --output-root /tmp/fly-smart-dy-implementation/accepted-matrix
```

### Validation record (2026-10-11)

- `uv sync --extra dev`, 56 pytest tests, component self-check and
  `git diff --check` passed. Core import separation and SVG XML checked.
- Final default CLI smoke: contact at 16.6625 s, y=0.01766 m, z=1.03806 m.
  CSV, summary and rendered plot are in `final-current-smoke` below.
- Godot normal-renderer smoke: contact at 17.4125 s, z=1.03169 m;
  headless editor parsing passed. This checks the camera adapter, not a
  repeated Godot disturbance matrix.
- All three final batches completed 80/80 centrally before edge-case audit.
  The final acceptance result below substitutes current-code repeats for
  decisions changed by the undershoot fix.

Run artifacts are under `/tmp/fly-smart-dy-implementation/`: `final-matrix-50`,
`final-matrix-75`, `final-matrix-100`, `undershoot-reruns`, `edge-case-repeats`
and `final-aggregate.json`. They are generated data, not repository source.
Each case saves settings, telemetry and summary. The aggregate has one result
for each unique distance/seed/case and identifies its source directory.

| Distance | Baseline central | Disturbance central | Worst lateral error | Worst altitude error |
|---|---:|---:|---:|---:|
| 50 m | 10/10 | 70/70 | 0.317 m | 0.093 m |
| 75 m | 10/10 | 70/70 | 0.108 m | 0.185 m |
| 100 m | 10/10 | 70/70 | 0.057 m | 0.106 m |

Acceptance passed: 30/30 baseline and 210/210 separate disturbance cases.
The full control-tick audit found three cases where the final undershoot
feasibility correction changed a decision: 50 m headwind seeds 3 and 4,
and 100 m headwind seed 9. All passed centrally on current-code repeats.
The aggregate also substitutes three additional current-code headwind runs
at 50 m (seeds 0..2). No trace exercised the fast-recovery-grace correction;
its regression test directly covers that branch.

Reference slew at 120 Hz remained <=1 m/s² (floating-point tolerance); no
approach collective-saturation ticks were reported. The measured peak downward
speed was approximately 2.433 m/s. These diagnostics are not physical braking
or motor-allocation guarantees. Case durations ranged from 15.86 to 57.07 s.

No combined-disturbance, moving/multiple-target, real-camera, hardware or full
seven-inch qualification was performed. The scoring proxy and simulated sensor
noise model limit what the 240 passing runs establish.

The seven-inch compatibility smoke exposed a profile-specific takeoff regression:
its old weak altitude PID settled several metres above 15 m, so the new explicit
altitude corridor delayed acquisition until battery voltage changed. Tune only
that shipped profile's altitude PID/integral authority against the same synthetic
sensors. Keep the default-vehicle controller and the frozen acceptance matrix
unchanged. The profile altitude tune (1.8,0.4,2.2), altitude integral limit 10 and
vertical-speed integral limit 10 restored acquisition/contact. The separate
smoke contacted at t=26.13 s, y=-0.009 m, z=0.932 m; it is outside the matrix.
Its takeoff overshoot remains approximately 18.67 m and needs separate vehicle
qualification.

The first broad matrix completed all 80 cases each at 50 and 75 m centrally,
but 100 m crosswind showed weak early lateral correction: drift reached about
6.7 m, putting a context building in the optical line of sight. A correct
bearing-loss abort followed. Increase lateral outer PD from (0.4,0,3.0) to
(0.8,0,4.0), keeping the tested inner roll PID, so the far-field error is
corrected sooner with strong damping. Do not remove occluders or relax loss
limits to conceal it. Validate the 100 m crosswind smoke, then rerun complete
50/75/100 m batches against the new defaults. The earlier partial matrix is
retained; it is not an acceptance result.

Final edge-case review: feasibility must use absolute altitude error and motion
signed toward the target altitude. Otherwise an undershoot >0.2 m is incorrectly
classified as an already-level finish. Use the climb limit for upward recovery,
descent limit for downward motion, and require abs(z-target_z)<=0.2 m for the
stable terminal exemption. Reject measured toward-target speed above its limit.
Add undershoot/overspeed regression tests and audit matrix traces for cases
that reach this corrected branch before declaring the matrix representative.

Recovery grace review: retain the entire preceding speed reference during the
capture-anchored grace interval, including TTC references below -2 m/s. Apply
the altitude-hold correction cap only when computing altitude hold; the shared
planner already bounds retained references. Add a regression at -4 m/s and
check matrix traces for any grace interval affected by this correction.

2026-10-12 follow-up: [150 m detection and bbox alignment](150m-target-detection-bbox.md)
changes the minimum resolved bearing patch to 3x3 while preserving all TTC
quality gates and controller tunings. Its two 150 m baseline smokes supplement
the earlier matrix; the full disturbance matrix was not rerun for that change.

2026-10-12 speed follow-up: [horizontal approach tuning](faster-horizontal-approach.md)
raises the default pitch request to 7 degrees, retaining 5 degrees explicitly
for the seven-inch profile. Earlier 240-case evidence used 5 degrees; the new
record lists the narrower validation scope, speed gains and 150 m crosswind
limitation. Descent laws and confidence gates are unchanged.
