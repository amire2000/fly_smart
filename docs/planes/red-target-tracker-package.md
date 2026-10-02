# Red-target tracker package

The reusable RGB red-target detector lives under `fly_smart.trackers`, along
with future target/state tracking components. Its canonical import is
`fly_smart.trackers.red_target_detector.detect_red_box`.

`fly_smart.red_target_detector` remains a compatibility import so existing
applications do not break while callers migrate. Detection behavior and the
function signature are unchanged.
