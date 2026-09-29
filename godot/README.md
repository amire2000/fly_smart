# Fly Smart Godot renderer

This project is the renderer for the `godot-render` branch. It receives
PyBullet drone and target poses on UDP `127.0.0.1:9100`. Its FPV `SubViewport`
is copied as RGB8 frames into `/dev/shm/fly_smart_fpv.rgb` using the working
double-buffered format from `godot_shm_fpv`.

Run the Godot scene before Python:

```bash
godot --path godot
uv run fly-smart --godot --headless
```

Godot is the visual camera only. Python remains responsible for OpenCV red
detection, TTC filtering, guidance, motor commands, and PyBullet physics.
