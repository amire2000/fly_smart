# 150 m detection and Godot bbox alignment

Implementation commit: `690d24e`.

## Reproduced failures

The real Godot scene displays the source frame in a 480x270 TextureRect but
maps both bbox axes with a fixed factor 0.75. For the default 640x480 capture,
y=300 appears at y=235 rather than y=178.75. The scene regression test fails
on this actual overlay method. Width/height are runtime camera settings.

A 2 m target rendered 150 m away by the default 640x480, 90-degree horizontal
camera produces a 5x5 pixel red patch (contour area 16). The detector rejects
contour area <20. A tilted 150 m camera regression also fails detection.

## Changes and control impact

Keep the rigid body-mounted sensor camera and its calibrated projection.
The camera does not point at simulator target coordinates: guidance uses the
measured compensated bearing. Automatic gimbal locking would require a
different sensor/attitude model and is outside this fix.

Map bbox position and size through the actual preview position and independent
preview/source width and height scales. Set explicit stretched preview behavior
so this transform matches the displayed texture at every configured resolution.

Permit resolved compact distant red patches: retain HSV thresholds and 3x3
morphological opening, accept contour area >=4 and bounding dimensions >=3.
Allow those same >=3x3 observations through bearing validity. Single-pixel and
2-pixel-wide red noise remains rejected. The smaller image is less precise;
bearing association/freshness and all TTC growth/window/fit gates are unchanged.
A small accepted box supplies bearing, not automatically reliable TTC. This is
a detection/bearing behavior change; no PID or state-machine tuning changes.

## Validation

- `uv run pytest -q tests/test_camera_adapters.py -k far_target` reproduces
  150 m rejection before the fix; 100 m already passes.
- `godot --headless --path godot --script ../tests/godot/test_fpv_overlay.gd`
  exercises actual scene bbox geometry at 640x480, 640x360 and 1280x960.
- Add small-patch/noise checks and rerun pytest and the component self-check.
- Render an actual Godot 150 m target, verify detection/compensated bearing,
  and run a baseline 150 m headless approach. Record results below.

No full 150 m disturbance matrix or hardware qualification is implied. Accepting
smaller red blobs increases susceptibility to compact red distractors; the
single-static-red-target assumption remains.

## Results (2026-10-12)

62 pytest tests, component self-check and diff whitespace checks passed. The
Godot overlay regression passed for ordinary and 3x3/4x4 distant boxes at all
three source resolutions. The harness calls the real overlay methods without
starting UDP/shared-memory renderer services.

Actual Godot 150 m RGB capture detected bbox (318,268,4,4): compensated
elevation 5.356 degrees, azimuth zero, bearing valid and TTC initially invalid.
The snapshot is `/tmp/fly-smart-150m-godot-detection.png`.

PyBullet baseline, seed 0: central contact at 44.15 s, y=-0.00534 m, z=0.93406 m.
Artifacts: `/tmp/fly-smart-150m-detection-smoke/`. Actual Godot camera baseline:
contact at 44.4708 s, y=-0.0000394 m, z=0.98088 m. Artifacts:
`/tmp/fly-smart-150m-godot-smoke/`. Both start 150 m from the 2 m target, with
default vehicle, camera and sensor settings. No disturbances were combined or
qualified in these two baseline smokes.
