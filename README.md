# Fly Smart

Standalone TTC diagonal-strike simulation extracted from the course.

The `master` branch is the PyBullet-only baseline. It keeps the current
course camera, physics, sensors, guidance, telemetry, and command-line entry
point. The `godot-render` branch replaces only the renderer: PyBullet remains
the physics authority, while Godot renders the scene and publishes its FPV
camera frames through Linux shared memory.

## Run the PyBullet baseline

```bash
uv sync
uv run python ttc_diagonal_strike.py --config configs/scenario.yaml
```

Useful checks:

```bash
uv run python ttc_diagonal_strike.py --self-check
uv run python ttc_diagonal_strike.py --headless --config configs/scenario.yaml
```

Each run is written to `outputs/ttc_runs/<run-name>/` with settings, CSV
telemetry, summary, and plots.

## Branches

| Branch | Renderer | Camera source |
| --- | --- | --- |
| `master` | PyBullet GUI | PyBullet camera |
| `godot-render` | Godot | Godot FPV camera via `/dev/shm/fly_smart_fpv.rgb` |

The course repository is the source of the baseline, but it is not imported
at runtime and is not modified by this project.
