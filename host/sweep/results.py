"""
Results I/O and summarization.

Writes full results (all trials) and compact summaries (top-N + stats)
designed for minimal Claude token consumption.
"""

import json
import os
from datetime import datetime


def write_full_results(ranked: list[dict], output_dir: str,
                       sweep_name: str, round_num: int) -> str:
    """Write all trial results to a JSON file.

    Returns the file path.
    """
    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f"{sweep_name}_round{round_num}_full.json")
    with open(path, "w") as f:
        json.dump(ranked, f, indent=1)
    return path


def write_summary(ranked: list[dict], output_dir: str,
                  sweep_name: str, round_num: int,
                  scenario: str, param_ranges: list,
                  scoring_config: dict,
                  top_n: int = 10) -> str:
    """Write a compact summary file.

    This is the file Claude reads — kept small.
    Returns the file path.
    """
    os.makedirs(output_dir, exist_ok=True)

    total = len(ranked)
    passing = [r for r in ranked if r["passed"]]
    pass_rate = len(passing) / total if total > 0 else 0.0

    # Top N
    top = passing[:top_n]
    top_entries = []
    for r in top:
        top_entries.append({
            "rank": r["rank"],
            "score": r["score"],
            "params": {k: round(v, 6) if isinstance(v, float) else v
                       for k, v in r["params"].items()},
            "metrics": r["metrics"],
        })

    # Param stats from top results
    param_stats = {}
    if top:
        all_param_names = list(top[0]["params"].keys())
        for pname in all_param_names:
            vals = [r["params"][pname] for r in top]
            param_stats[pname] = {
                "min": round(min(vals), 6),
                "max": round(max(vals), 6),
                "mean": round(sum(vals) / len(vals), 6),
            }

    # Recommended next ranges (convex hull of top 20% + 10% expansion)
    top_20pct = passing[:max(1, len(passing) // 5)]
    recommended = {}
    for pr in param_ranges:
        vals = [r["params"][pr.name] for r in top_20pct]
        if vals:
            lo, hi = min(vals), max(vals)
            span = hi - lo
            margin = max(span * 0.1, pr.span * 0.01)
            recommended[pr.name] = [
                round(max(pr.low, lo - margin), 6),
                round(min(pr.high, hi + margin), 6),
            ]

    # Failure analysis
    failing = [r for r in ranked if not r["passed"]]
    fail_reasons = {}
    for r in failing:
        reason = r.get("fail_reason", "unknown")
        fail_reasons[reason] = fail_reasons.get(reason, 0) + 1

    summary = {
        "sweep": sweep_name,
        "scenario": scenario,
        "round": round_num,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "samples": total,
        "passed": len(passing),
        "pass_rate": round(pass_rate, 3),
        "scoring": scoring_config,
        "top_n": top_entries,
        "param_stats_top10": param_stats,
        "recommended_next_range": recommended,
        "fail_reasons": fail_reasons,
    }

    path = os.path.join(output_dir, f"{sweep_name}_round{round_num}_summary.json")
    with open(path, "w") as f:
        json.dump(summary, f, indent=2)
    return path


def write_best_params(ranked: list[dict], output_dir: str,
                      sweep_name: str) -> str:
    """Write the single best parameter set to a file."""
    os.makedirs(output_dir, exist_ok=True)

    passing = [r for r in ranked if r["passed"]]
    if not passing:
        best = {"error": "no passing trials"}
    else:
        r = passing[0]
        best = {
            "sweep": sweep_name,
            "score": r["score"],
            "params": r["params"],
            "metrics": r["metrics"],
        }

    path = os.path.join(output_dir, f"{sweep_name}_best.json")
    with open(path, "w") as f:
        json.dump(best, f, indent=2)
    return path


def load_summary(path: str) -> dict:
    """Load a summary file."""
    with open(path) as f:
        return json.load(f)
