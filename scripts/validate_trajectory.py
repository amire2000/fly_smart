"""Reproducible sensor-only trajectory matrix; truth is used only for scoring."""

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, replace
import json
from pathlib import Path

import cv2
import pybullet as p

from fly_smart.simulation.config_loader import load_yaml_config
from fly_smart.simulation.runner import StrikeSimulation


CASES = ("baseline", "offset_left", "offset_right", "wind_x_plus", "wind_x_minus",
         "wind_y_plus", "wind_y_minus", "dropout")


def run_case(job):
    """Run one isolated physics connection and score contact near the target centre."""
    distance, seed, case, output_root, config_path = job
    cv2.setNumThreads(1)
    directory = Path(output_root) / f"{distance}m-seed{seed}-{case}"
    directory.mkdir(parents=True, exist_ok=False)
    config = load_yaml_config(Path(config_path))
    target = (config.launch_position[0] + distance,
              5.0 if case == "offset_left" else (-5.0 if case == "offset_right" else 0.0),
              config.impact_altitude_m)
    winds = {"wind_x_plus": (2., 0., 0.), "wind_x_minus": (-2., 0., 0.),
             "wind_y_plus": (0., 2., 0.), "wind_y_minus": (0., -2., 0.)}
    scene = replace(config.simulation, target_center=target, random_seed=seed,
                    post_impact_seconds=0., wind_world_mps=winds.get(case, (0., 0., 0.)),
                    camera_dropout_intervals_s=((9., 9.2),) if case == "dropout" else ())
    config = replace(config, simulation=scene)
    (directory / "settings.json").write_text(json.dumps({"simulation": asdict(scene), "runtime": asdict(config.runtime)}, indent=2) + "\n")
    client = p.connect(p.DIRECT)
    try:
        result = StrikeSimulation(config).run(False, 60., None, None,
            directory / "telemetry.csv", directory / "summary.json")
    finally:
        p.disconnect(client)
    summary = json.loads((directory / "summary.json").read_text())
    position = summary["collision"]["position_m"]
    half = scene.target_size_m / 4
    central = bool(result.success and abs(position[1] - target[1]) <= half and abs(position[2] - target[2]) <= half)
    return {"distance_m": distance, "seed": seed, "case": case, "contact": result.success,
            "central_contact": central, "position_m": position, "abort_reason": summary["abort_reason"],
            "time_s": result.simulated_time_s}


def main():
    """Write every outcome and return a failing exit code below acceptance targets."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/scenario.yaml")
    parser.add_argument("--seeds", type=int, default=10)
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--distances", type=int, nargs="+", default=[50, 75, 100])
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    parser.add_argument("--output-root", type=Path, required=True)
    args = parser.parse_args()
    if args.seeds < 1 or args.workers < 1:
        parser.error("seeds and workers must be positive")
    args.output_root.mkdir(parents=True, exist_ok=False)
    jobs = [(d, seed, case, str(args.output_root), args.config)
            for d in args.distances for seed in range(args.seeds) for case in args.cases]
    results = []
    with ProcessPoolExecutor(args.workers) as executor:
        for future in as_completed([executor.submit(run_case, job) for job in jobs]):
            row = future.result()
            results.append(row)
            print(json.dumps(row), flush=True)
            (args.output_root / "matrix.json").write_text(json.dumps(results, indent=2) + "\n")
    baseline = [r for r in results if r["case"] == "baseline"]
    disturbed = [r for r in results if r["case"] != "baseline"]
    passed = all(r["central_contact"] for r in baseline) and (not disturbed or sum(r["central_contact"] for r in disturbed) / len(disturbed) >= .95)
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
