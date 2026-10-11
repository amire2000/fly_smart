# Trajectory algorithm documentation plan

Create `docs/trajectory_algo.md` from the current implementation so future
control design has an explicit sensor-to-command baseline.

Scope: trace takeoff completion, TRACK/DY, TRACK/TTC, target loss, COMMIT,
ABORT, attitude control, motor allocation, and target contact. Explain sensor
timing, equations, command ownership, configured defaults, and control limits.
Include Mermaid flow and sequence diagrams, source links, and distinguish
observed code behavior from future design considerations.

Read the core guidance, trajectory, TTC, camera geometry, fusion and PID code,
their simulation callers, scenario configuration, and existing tests. Validate
the explanation against those tests and component self-check; exercise the
default headless scenario if the environment permits. Check documentation links.

Deliver only documentation and index entries. Preserve existing uncommitted
documents and index entries. No guidance, TTC, tuning, configuration, API,
fixture, generated run data, or release changes are planned.

Implementation commit: `pending`.
