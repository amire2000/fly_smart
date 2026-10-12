# Faster horizontal approach

Implementation commit: `pending`. Implemented; simulation checks recorded below.

## Intent and current control

Increase approach horizontal speed while preserving bearing-first descent,
TTC quality gates, braking limits and accuracy priority. Guidance presently
requests 5 degrees forward pitch; neither legacy forward_speed_mps=13 nor
nominal_pitch_deg=20 is an active horizontal-speed command. No measured metric
forward speed enters guidance, so a pitch change is an acceleration/approach
request and cannot guarantee a particular speed under wind or drag.

## Candidate and validation

Evaluate 7 and 8 degree pitch requests using temporary scenario files before
selecting a shipped setting. Keep pitch slew, vertical/lateral PID gains,
confidence thresholds and speed limits unchanged. Compare 50/100/150 m baseline
runs at 15 m takeoff with the known roughly 4 m/s approach. Inspect contact,
central-contact error, duration, recovery/abort and peak/impact speed. Reject
candidates that shorten available descent time beyond the feasibility screen.
After choosing a setting, check separate offset/wind/dropout cases and document
validation scope. Higher takeoff presets require independent qualification.
No optional speed preference was received during the initial tests. Proceed
with a moderate increase and preserve accuracy priority: 7 degrees passed the
30/50/100/150 m seed-0 baselines; 8 degrees failed the 30 m feasibility screen.
Evaluate 7 degrees against near-distance noise seeds and far disturbances
before shipping it. Do not change braking limits to make 8 degrees pass.

Also check 150 m baseline approaches at the user-selected 20/30/50 m takeoff
altitudes with the candidate. These are baseline compatibility checks, not a
higher-altitude disturbance matrix.

The 7-degree 150 m baselines at 20/30/50 m takeoff all contacted centrally.
The separate 150 m +Y wind case lost bearing and aborted at 12.17 s; compare
it with the original 5-degree configuration before attributing the loss to
the speed change. Keep loss-abort gates intact.

## Selection and shipped scope

Select 7 degrees as the moderate default. Core MissionConfig, default scenario
and template must agree. Preserve the seven-inch profile's separately tuned
5-degree request explicitly; no heavier-vehicle speed qualification was run.
Update current control documentation and stage schematic; historical tuning
records remain historical. Pitch slew remains 5 degrees/s.

The 150 m +Y wind case also fails bearing retention at 5 degrees (13.23 s),
versus 12.17 s at 7 degrees. This is a known far-field bearing-loss limitation,
not evidence of a new speed regression. Do not conceal it or raise loss limits.

## Validation results (2026-10-12)

Selected 7-degree seed-0 baseline results at 15 m takeoff:

| Distance | Contact time | Forward speed at contact | Central contact |
|---|---:|---:|---|
| 30 m | 14.48 s | 4.84 m/s | yes |
| 50 m | 18.18 s | 5.44 m/s | yes |
| 100 m | 27.10 s | 5.63 m/s | yes |
| 150 m | 35.94 s | 5.65 m/s | yes |

Ten 30 m baseline seeds all scored centrally. At 50/100/150 m, one seed per
eight separate cases (baseline, +/-5 m offset, +/-2 m/s X/Y wind and 0.2 s
camera dropout) scored 23/24 centrally. All 16 cases at 50 and 100 m passed;
150 m passed 7/8, failing +Y crosswind bearing retention at both old/new pitch.
Of the 21 disturbed cases, 20/21 passed (95.24%); this is a small one-seed
sample, not a repeat of the earlier 240-case qualification.

At 150 m, baseline 20/30/50 m takeoff presets all passed centrally with
contact times 36.54/37.84/41.34 s and contact altitudes 0.992/0.988/1.033 m.
The current-config Godot 30 m smoke contacted at 14.875 s, z=1.0837 m,
y approximately zero and horizontal speed 4.934 m/s. A fixed pitch request
does not guarantee these speeds under other wind/vehicle conditions.

72 pytest tests, component self-check and diff checks passed. Artifacts are
under `/tmp/fly-smart-faster-pitch-7-*`, `/tmp/fly-smart-faster-pitch-8*`,
`/tmp/fly-smart-original-pitch-150-crosswind` and
`/tmp/fly-smart-faster-godot-smoke`. No hardware or higher-altitude disturbance
matrix was tested. Do not recommend 8 degrees globally: its 30 m baseline
aborts on descent infeasibility rather than finishing the descent.
