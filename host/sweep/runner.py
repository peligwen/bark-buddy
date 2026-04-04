#!/usr/bin/env python3
"""
Parameter sweep runner — main entry point.

Orchestrates recursive coarse-to-fine parameter sweeps using Latin Hypercube
Sampling across 8 parallel worker processes.

Usage:
    python -m sweep.runner --sweep gait --scenario flat_walk
    python -m sweep.runner --sweep gait --scenario flat_walk --samples 50 --rounds 3
    python -m sweep.runner --sweep balance_pid --scenario push_recovery
    python -m sweep.runner --list-sweeps
    python -m sweep.runner --list-scenarios
"""

import argparse
import json
import multiprocessing
import os
import sys
import time

# Ensure host/ is on path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from sweep.sampler import ParamRange, latin_hypercube, narrow_ranges
from sweep.scoring import score_trials, SCORING_PRESETS
from sweep.results import write_full_results, write_summary, write_best_params
from sweep.worker import run_trial

# ---- Parameter space presets ----

SWEEP_PRESETS = {
    "gait": [
        ParamRange("hip_amplitude", 0.06, 0.28),
        ParamRange("knee_amplitude", 0.03, 0.18),
        ParamRange("lift_height", 0.004, 0.025),
        ParamRange("gait_frequency", 0.8, 3.0),
    ],
    "balance_pid": [
        ParamRange("joint_kp", 5.0, 50.0),
        ParamRange("joint_kd", 0.5, 8.0),
        ParamRange("max_torque", 0.3, 3.0),
        ParamRange("contact_k", 300.0, 1500.0),
    ],
    "contact": [
        ParamRange("contact_k", 200.0, 2000.0),
        ParamRange("contact_c", 10.0, 100.0),
        ParamRange("contact_mu", 0.3, 1.5),
    ],
    "joint_servo": [
        ParamRange("joint_kp", 5.0, 60.0),
        ParamRange("joint_kd", 0.2, 10.0),
        ParamRange("max_torque", 0.2, 4.0),
    ],
    "movement": [
        ParamRange("forward_speed", 0.03, 0.25),
        ParamRange("turn_speed", 15.0, 90.0),
        ParamRange("hip_amplitude", 0.06, 0.28),
        ParamRange("gait_frequency", 0.8, 3.0),
    ],
    "full": [
        ParamRange("hip_amplitude", 0.06, 0.28),
        ParamRange("knee_amplitude", 0.03, 0.18),
        ParamRange("lift_height", 0.004, 0.025),
        ParamRange("gait_frequency", 0.8, 3.0),
        ParamRange("joint_kp", 5.0, 50.0),
        ParamRange("joint_kd", 0.5, 8.0),
    ],
}

# Default scenario configs (can be overridden via --config)
DEFAULT_SCENARIO_CONFIGS = {
    "flat_walk": {"duration": 5.0},
    "push_recovery": {"push_force": 0.15, "walk_time_before": 1.5,
                       "max_time": 3.0, "settle_threshold": 5.0},
    "slope_climb": {"slope_deg": 10.0, "duration": 5.0},
    "obstacle_crossing": {"bump_height_mm": 5.0, "bump_x_start": 0.15,
                          "bump_x_end": 0.25, "duration": 5.0},
    "cliff_detect": {"cliff_x": 0.30, "duration": 5.0, "sonar_check_hz": 20},
    "turn_accuracy": {"target_deg": 90.0, "direction": 5, "max_time": 5.0},
    "start_stop": {"walk_time": 3.0, "coast_time": 2.0},
    "sustained_walk": {"duration": 30.0},
}


def run_sweep(sweep_name: str, scenario_name: str,
              n_samples: int = 200, max_rounds: int = 5,
              n_workers: int = 8, target_score: float = 0.9,
              expansion: float = 0.1, top_pct: float = 0.2,
              output_dir: str = None, scenario_config: dict = None,
              scoring_config: dict = None, seed: int = 42):
    """Run a recursive parameter sweep.

    Args:
        sweep_name: preset name from SWEEP_PRESETS or "custom"
        scenario_name: scenario function name from SCENARIOS
        n_samples: samples per round
        max_rounds: maximum refinement rounds
        n_workers: parallel worker processes
        target_score: stop if top score >= this
        expansion: range expansion factor for narrowing
        top_pct: fraction of passing results to use for narrowing
        output_dir: output directory (default: sweep_results/<sweep>_<scenario>/)
        scenario_config: override scenario configuration
        scoring_config: override scoring configuration
        seed: random seed for reproducibility
    """
    if sweep_name not in SWEEP_PRESETS:
        print(f"Unknown sweep preset: {sweep_name}")
        print(f"Available: {', '.join(SWEEP_PRESETS.keys())}")
        return

    ranges = [ParamRange(p.name, p.low, p.high) for p in SWEEP_PRESETS[sweep_name]]

    if scenario_config is None:
        scenario_config = DEFAULT_SCENARIO_CONFIGS.get(scenario_name, {})

    if scoring_config is None:
        scoring_config = SCORING_PRESETS.get(scenario_name, {
            "gates": [("fell", "==", False)],
            "objectives": [("distance", "maximize", 1.0)],
        })

    if output_dir is None:
        output_dir = os.path.join("sweep_results", f"{sweep_name}_{scenario_name}")

    print(f"=== Parameter Sweep: {sweep_name} / {scenario_name} ===")
    print(f"Samples/round: {n_samples}, max rounds: {max_rounds}, workers: {n_workers}")
    print(f"Parameters: {[p.name for p in ranges]}")
    print(f"Target score: {target_score}")
    print()

    best_score_overall = -1.0
    stagnant_rounds = 0

    for round_num in range(1, max_rounds + 1):
        print(f"--- Round {round_num} ---")
        for p in ranges:
            print(f"  {p.name}: [{p.low:.6f}, {p.high:.6f}]")

        # Generate samples
        samples = latin_hypercube(ranges, n_samples, seed=seed + round_num)

        # Build worker args
        worker_args = [
            (i, sample, scenario_name, scenario_config)
            for i, sample in enumerate(samples)
        ]

        # Run in parallel
        t0 = time.time()
        with multiprocessing.Pool(processes=n_workers) as pool:
            results = pool.map(run_trial, worker_args)
        elapsed = time.time() - t0

        # Score and rank
        ranked = score_trials(results, scoring_config)

        # Write results
        full_path = write_full_results(ranked, output_dir, sweep_name, round_num)
        summary_path = write_summary(
            ranked, output_dir, sweep_name, round_num,
            scenario_name, ranges, scoring_config,
        )

        # Stats
        passing = [r for r in ranked if r["passed"]]
        pass_rate = len(passing) / len(ranked)
        top_score = passing[0]["score"] if passing else -1.0

        print(f"  Completed in {elapsed:.1f}s")
        print(f"  Pass rate: {len(passing)}/{len(ranked)} ({pass_rate:.1%})")
        if passing:
            print(f"  Top score: {top_score:.4f}")
            top = passing[0]
            print(f"  Best params: {json.dumps({k: round(v, 4) for k, v in top['params'].items()})}")
            print(f"  Best metrics: {json.dumps(top['metrics'])}")
        print(f"  Summary: {summary_path}")
        print()

        # Check termination conditions
        if top_score >= target_score:
            print(f"Target score {target_score} reached! Stopping.")
            break

        if top_score <= best_score_overall + 0.001:
            stagnant_rounds += 1
            if stagnant_rounds >= 2:
                # Widen expansion to escape local region
                expansion = min(expansion * 2, 0.5)
                print(f"  Score stagnant for {stagnant_rounds} rounds, widening expansion to {expansion}")
        else:
            stagnant_rounds = 0

        best_score_overall = max(best_score_overall, top_score)

        # Narrow ranges for next round
        if round_num < max_rounds and passing:
            top_n = max(1, int(len(passing) * top_pct))
            top_params = [r["params"] for r in passing[:top_n]]
            ranges = narrow_ranges(ranges, top_params, expansion=expansion)

            # Check if ranges have collapsed
            all_narrow = all(p.span < 1e-6 for p in ranges)
            if all_narrow:
                print("Ranges collapsed — stopping.")
                break

    # Write best params
    all_passing = [r for r in ranked if r["passed"]]
    if all_passing:
        best_path = write_best_params(ranked, output_dir, sweep_name)
        print(f"Best params written to: {best_path}")

    print("Done.")


def main():
    parser = argparse.ArgumentParser(
        description="MechDog parameter sweep tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python -m sweep.runner --sweep gait --scenario flat_walk
  python -m sweep.runner --sweep gait --scenario flat_walk --samples 50 --rounds 2
  python -m sweep.runner --sweep balance_pid --scenario push_recovery --workers 4
  python -m sweep.runner --sweep full --scenario sustained_walk --samples 300
  python -m sweep.runner --list-sweeps
  python -m sweep.runner --list-scenarios
        """,
    )
    parser.add_argument("--sweep", type=str, help="Sweep preset name")
    parser.add_argument("--scenario", type=str, help="Scenario name")
    parser.add_argument("--samples", type=int, default=200,
                        help="Samples per round (default: 200)")
    parser.add_argument("--rounds", type=int, default=5,
                        help="Max refinement rounds (default: 5)")
    parser.add_argument("--workers", type=int, default=8,
                        help="Parallel worker processes (default: 8)")
    parser.add_argument("--target-score", type=float, default=0.9,
                        help="Stop when top score >= this (default: 0.9)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed (default: 42)")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Output directory")
    parser.add_argument("--config", type=str, default=None,
                        help="JSON file with scenario config overrides")
    parser.add_argument("--list-sweeps", action="store_true",
                        help="List available sweep presets")
    parser.add_argument("--list-scenarios", action="store_true",
                        help="List available scenarios")

    args = parser.parse_args()

    if args.list_sweeps:
        print("Available sweep presets:")
        for name, ranges in SWEEP_PRESETS.items():
            params = ", ".join(f"{p.name}[{p.low}-{p.high}]" for p in ranges)
            print(f"  {name}: {params}")
        return

    if args.list_scenarios:
        from sweep.scenarios import SCENARIOS
        print("Available scenarios:")
        for name in SCENARIOS:
            config = DEFAULT_SCENARIO_CONFIGS.get(name, {})
            print(f"  {name}: {json.dumps(config)}")
        return

    if not args.sweep or not args.scenario:
        parser.error("--sweep and --scenario are required (or use --list-sweeps/--list-scenarios)")

    scenario_config = None
    if args.config:
        with open(args.config) as f:
            scenario_config = json.load(f)

    run_sweep(
        sweep_name=args.sweep,
        scenario_name=args.scenario,
        n_samples=args.samples,
        max_rounds=args.rounds,
        n_workers=args.workers,
        target_score=args.target_score,
        output_dir=args.output_dir,
        scenario_config=scenario_config,
        seed=args.seed,
    )


if __name__ == "__main__":
    main()
