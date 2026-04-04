"""
Latin Hypercube Sampling with coarse-to-fine recursive narrowing.

Each parameter dimension is divided into N equal strata, and exactly one
sample is drawn from each stratum (per dimension), then columns are
shuffled independently to form the LHS design.
"""

import random
from dataclasses import dataclass


@dataclass
class ParamRange:
    """A single parameter's sweep range."""
    name: str
    low: float
    high: float

    @property
    def span(self):
        return self.high - self.low


def latin_hypercube(ranges: list[ParamRange], n_samples: int,
                    seed: int | None = None) -> list[dict[str, float]]:
    """Generate n_samples points via Latin Hypercube Sampling.

    Returns list of dicts, each mapping param name → value.
    """
    rng = random.Random(seed)
    n_params = len(ranges)

    # For each param, create stratified samples then shuffle
    columns = []
    for p in ranges:
        strata = []
        for i in range(n_samples):
            lo = p.low + (i / n_samples) * p.span
            hi = p.low + ((i + 1) / n_samples) * p.span
            strata.append(rng.uniform(lo, hi))
        rng.shuffle(strata)
        columns.append(strata)

    # Assemble into list of param dicts
    samples = []
    for i in range(n_samples):
        sample = {}
        for j, p in enumerate(ranges):
            sample[p.name] = columns[j][i]
        samples.append(sample)

    return samples


def narrow_ranges(ranges: list[ParamRange],
                  top_params: list[dict[str, float]],
                  expansion: float = 0.1) -> list[ParamRange]:
    """Narrow parameter ranges to the convex hull of top results, +expansion.

    Args:
        ranges: current parameter ranges
        top_params: list of param dicts from the top-scoring trials
        expansion: fraction of new span to expand on each side (0.1 = 10%)

    Returns:
        New list of ParamRange narrowed around the top results.
    """
    new_ranges = []
    for p in ranges:
        values = [d[p.name] for d in top_params]
        lo = min(values)
        hi = max(values)
        span = hi - lo
        if span < 1e-12:
            # All top results at same value — expand around it
            span = p.span * 0.1
            lo -= span / 2
            hi += span / 2
        else:
            margin = span * expansion
            lo -= margin
            hi += margin

        # Clamp to original bounds
        lo = max(lo, p.low)
        hi = min(hi, p.high)

        new_ranges.append(ParamRange(name=p.name, low=lo, high=hi))

    return new_ranges
