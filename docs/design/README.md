# Design document index

Keep this table current when adding or changing a design document. The commit
column names the commit that implemented the documented change; use `pending`
for work that has not been committed yet.

| Document | Description | Implementing commit |
| --- | --- | --- |
| [godot-interactive-attempt-controls.md](godot-interactive-attempt-controls.md) | Interactive start/reset lifecycle, attempt outputs, and Godot pose reset. | `e208f57` |
| [godot-interactive-controls.md](godot-interactive-controls.md) | Godot toolbar buttons, UDP control events, and the Python/PyBullet flow. | `pending` |
| [interactive-stop-exit-status.md](interactive-stop-exit-status.md) | Treat operator Stop as a clean interactive exit after saving artifacts. | `pending` |
| [godot-fpv-telemetry-overlay.md](godot-fpv-telemetry-overlay.md) | Draw Python red-target bbox and live flight telemetry over Godot's FPV texture. | `pending` |
| [godot-live-telemetry-plot.md](godot-live-telemetry-plot.md) | Optional UDP-fed live telemetry plot for Godot-backed runs. | `pending` |
| [godot-opencv-preview.md](godot-opencv-preview.md) | Godot FPV overlay replacing the former OpenCV preview window. | `pending` |
| [godot-pybullet-renderer-scene.md](godot-pybullet-renderer-scene.md) | Godot renderer scene, pose bridge, FPV camera, and collision events. | `e208f57` |
| [godot-rtf-overlay.md](godot-rtf-overlay.md) | Calculate session real-time factor and show it in Godot's lower-right overlay. | `pending` |
| [live-plot-process.md](live-plot-process.md) | Move live Matplotlib plotting to a UDP-fed subprocess so it cannot block physics. | `pending` |
| [godot-target-distance-selector.md](godot-target-distance-selector.md) | Reset-time Godot target distance presets. | `pending` |
| [godot-cpu-bbox-diagnostics.md](godot-cpu-bbox-diagnostics.md) | Prevent plotter busy-loop CPU use and retain distant red-target detections. | `pending` |
| [live-plot-target-distance.md](live-plot-target-distance.md) | Update the live plot target marker after each reset distance selection. | `pending` |
| [godot-video-frame-size.md](godot-video-frame-size.md) | Normalize Godot camera frames before video encoding. | `e208f57` |
| [ttc-feedforward-descent.md](ttc-feedforward-descent.md) | Feed-forward descent behavior when TTC is unavailable. | `69fc584` |
| [ttc-unavailable-pitch-boost.md](ttc-unavailable-pitch-boost.md) | Default 5° pitch boost while a visible target lacks valid TTC. | `pending` |
| [ttc-trust-gate-and-descent-cap.md](ttc-trust-gate-and-descent-cap.md) | Ignore long TTC estimates and bound altitude-error descent correction. | `pending` |
| [ttc-seconds-plot.md](ttc-seconds-plot.md) | Display raw and filtered TTC estimates in seconds with the activation gate. | `pending` |
| [bbox-center-alignment-plot.md](bbox-center-alignment-plot.md) | Plot bbox-center pixel errors and calibrated angular offsets. | `pending` |
| [camera_pitch_compensation.md](camera_pitch_compensation.md) | Pitch-compensated bbox vertical error and DY/TTC altitude control. | `pending` |
| [max-descent-rate-5_5.md](max-descent-rate-5_5.md) | Raise the shared maximum descent-rate limit to 5.5 m/s. | `pending` |
| [vscode-godot-task.md](vscode-godot-task.md) | VS Code tasks for Godot, Python, and the compound launcher. | `e208f57` |
