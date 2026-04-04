"""
Scoring system: boolean pass/fail gates + weighted multi-objective ranking.

Gates are checked first — any failure disqualifies the trial.
Objectives are normalized 0-1 within the round and weighted.
"""


def check_gates(metrics: dict, gates: list[tuple]) -> tuple[bool, str | None]:
    """Check boolean pass/fail gates.

    Args:
        metrics: trial metrics dict
        gates: list of (metric_name, operator, threshold) tuples
            operators: "==", "!=", ">", ">=", "<", "<="

    Returns:
        (passed: bool, reason: str | None)
    """
    ops = {
        "==": lambda a, b: a == b,
        "!=": lambda a, b: a != b,
        ">": lambda a, b: a > b,
        ">=": lambda a, b: a >= b,
        "<": lambda a, b: a < b,
        "<=": lambda a, b: a <= b,
    }

    for metric_name, op, threshold in gates:
        if metric_name not in metrics:
            return False, f"missing metric: {metric_name}"
        value = metrics[metric_name]
        if not ops[op](value, threshold):
            return False, f"{metric_name}={value} failed {op} {threshold}"

    return True, None


def score_objectives(metrics: dict, objectives: list[tuple],
                     all_metrics: list[dict]) -> float:
    """Compute weighted score from objectives.

    Each objective is normalized 0-1 within the range of all_metrics
    for that metric, then weighted.

    Args:
        metrics: this trial's metrics
        objectives: list of (metric_name, "maximize"|"minimize", weight)
        all_metrics: metrics from all passing trials (for normalization)

    Returns:
        weighted score in [0, 1]
    """
    if not objectives or not all_metrics:
        return 0.0

    total_weight = sum(w for _, _, w in objectives)
    if total_weight == 0:
        return 0.0

    score = 0.0
    for metric_name, direction, weight in objectives:
        values = [m[metric_name] for m in all_metrics if metric_name in m]
        if not values:
            continue

        lo = min(values)
        hi = max(values)
        val = metrics.get(metric_name, lo)

        if hi - lo < 1e-12:
            normalized = 0.5
        elif direction == "maximize":
            normalized = (val - lo) / (hi - lo)
        else:  # minimize
            normalized = (hi - val) / (hi - lo)

        normalized = max(0.0, min(1.0, normalized))
        score += normalized * weight

    return round(score / total_weight, 6)


def score_trials(results: list[dict], scoring_config: dict) -> list[dict]:
    """Score and rank all trial results.

    Args:
        results: list of trial result dicts (from worker.run_trial)
        scoring_config: dict with "gates" and "objectives" keys

    Returns:
        results list with added "passed", "fail_reason", "score", "rank" keys,
        sorted by score descending (passing trials first).
    """
    gates = scoring_config.get("gates", [])
    objectives = scoring_config.get("objectives", [])

    # Check gates
    for r in results:
        passed, reason = check_gates(r["metrics"], gates)
        r["passed"] = passed
        r["fail_reason"] = reason

    # Collect passing metrics for normalization
    passing = [r for r in results if r["passed"]]
    passing_metrics = [r["metrics"] for r in passing]

    # Score passing trials
    for r in passing:
        r["score"] = score_objectives(r["metrics"], objectives, passing_metrics)

    for r in results:
        if not r["passed"]:
            r["score"] = -1.0

    # Sort: passing first (by score desc), then failing
    passing.sort(key=lambda r: r["score"], reverse=True)
    failing = [r for r in results if not r["passed"]]

    ranked = passing + failing
    for i, r in enumerate(ranked):
        r["rank"] = i + 1

    return ranked


# ---- Preset scoring configs per scenario ----

SCORING_PRESETS = {
    "flat_walk": {
        "gates": [
            ("fell", "==", False),
            ("distance", ">", 0.10),
        ],
        "objectives": [
            ("distance", "maximize", 0.5),
            ("max_pitch", "minimize", 0.3),
            ("energy", "minimize", 0.2),
        ],
    },
    "push_recovery": {
        "gates": [
            ("fell", "==", False),
        ],
        "objectives": [
            ("settle_time", "minimize", 0.6),
            ("max_tilt", "minimize", 0.4),
        ],
    },
    "slope_climb": {
        "gates": [
            ("fell", "==", False),
            ("made_progress", "==", True),
        ],
        "objectives": [
            ("net_distance", "maximize", 0.6),
            ("max_pitch", "minimize", 0.4),
        ],
    },
    "obstacle_crossing": {
        "gates": [
            ("fell", "==", False),
            ("crossed", "==", True),
        ],
        "objectives": [
            ("distance", "maximize", 0.4),
            ("max_pitch", "minimize", 0.3),
            ("max_roll", "minimize", 0.3),
        ],
    },
    "cliff_detect": {
        "gates": [
            ("fell_off", "==", False),
        ],
        "objectives": [
            ("overshoot", "minimize", 1.0),
        ],
    },
    "turn_accuracy": {
        "gates": [
            ("fell", "==", False),
        ],
        "objectives": [
            ("heading_error", "minimize", 0.6),
            ("max_pitch", "minimize", 0.2),
            ("max_roll", "minimize", 0.2),
        ],
    },
    "start_stop": {
        "gates": [
            ("fell", "==", False),
        ],
        "objectives": [
            ("overshoot", "minimize", 0.7),
            ("max_pitch_at_stop", "minimize", 0.3),
        ],
    },
    "sustained_walk": {
        "gates": [
            ("fell", "==", False),
            ("distance", ">", 0.5),
        ],
        "objectives": [
            ("distance", "maximize", 0.3),
            ("lateral_drift", "minimize", 0.2),
            ("yaw_drift", "minimize", 0.2),
            ("pitch_std", "minimize", 0.15),
            ("max_pitch", "minimize", 0.15),
        ],
    },
}
