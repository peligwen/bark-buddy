#!/usr/bin/env python3
"""
Apply gait_config.json parameters to firmware/include/config.h.

Reads tuned parameters from gait_config.json (produced by sweep runner
with --apply) and patches the corresponding #define values in config.h.

The firmware currently uses a single set of gait parameters for all
directions. By default, the forward gait is applied. Use --direction
to apply a different direction's params.

Usage:
    python3 apply_to_firmware.py                     # preview forward gait
    python3 apply_to_firmware.py --write             # write to config.h
    python3 apply_to_firmware.py --direction backward # apply backward gait
    python3 apply_to_firmware.py --all                # show all directions
"""

import argparse
import json
import math
import os
import re
import sys

GAIT_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "gait_config.json")
FIRMWARE_CONFIG_PATH = os.path.join(os.path.dirname(__file__),
                                     "..", "firmware", "include", "config.h")

# Map from gait_config.json keys to firmware #define names + unit conversion
# Sim uses radians, firmware uses degrees for gait angles
PARAM_TO_DEFINE = {
    "hip_amplitude": ("GAIT_HIP_AMPLITUDE", lambda v: f"{math.degrees(v):.1f}f",
                      "degrees"),
    "knee_amplitude": ("GAIT_KNEE_AMPLITUDE", lambda v: f"{math.degrees(v):.1f}f",
                       "degrees"),
    "gait_frequency": ("GAIT_FREQUENCY", lambda v: f"{v:.2f}f", "Hz"),
}

DIRECTIONS = ["forward", "backward", "turn_left", "turn_right"]


def load_config(path: str, direction: str = "forward") -> dict:
    """Load params for a specific direction from gait_config.json."""
    with open(path) as f:
        data = json.load(f)

    # Per-direction format
    directions = data.get("directions", {})
    if direction in directions:
        return directions[direction]

    # Legacy flat format
    return data.get("params", data)


def load_all_directions(path: str) -> dict:
    """Load all direction params from gait_config.json."""
    with open(path) as f:
        data = json.load(f)
    return data.get("directions", {})


def patch_config_h(config_h_text: str, params: dict) -> tuple[str, list[str]]:
    """Patch #define values in config.h text.

    Returns (new_text, list of change descriptions).
    """
    changes = []
    new_text = config_h_text

    for param_key, (define_name, formatter, unit) in PARAM_TO_DEFINE.items():
        if param_key not in params:
            continue

        value = params[param_key]
        new_value_str = formatter(value)

        pattern = rf"(#define\s+{define_name}\s+)(\S+)"
        match = re.search(pattern, new_text)
        if not match:
            changes.append(f"  WARNING: {define_name} not found in config.h")
            continue

        old_value_str = match.group(2)
        if old_value_str == new_value_str:
            changes.append(f"  {define_name}: {old_value_str} (unchanged)")
            continue

        new_text = re.sub(pattern, rf"\g<1>{new_value_str}", new_text)
        changes.append(f"  {define_name}: {old_value_str} -> {new_value_str} ({unit})")

    return new_text, changes


def main():
    parser = argparse.ArgumentParser(
        description="Apply gait_config.json to firmware config.h")
    parser.add_argument("--config", default=GAIT_CONFIG_PATH,
                        help="Path to gait config JSON")
    parser.add_argument("--firmware-config", default=FIRMWARE_CONFIG_PATH,
                        help="Path to firmware config.h")
    parser.add_argument("--direction", default="forward",
                        choices=DIRECTIONS,
                        help="Which direction's gait to apply (default: forward)")
    parser.add_argument("--write", action="store_true",
                        help="Write changes (default: preview only)")
    parser.add_argument("--all", action="store_true",
                        help="Show all tuned directions (no write)")
    args = parser.parse_args()

    if not os.path.exists(args.config):
        print(f"No config file at {args.config}")
        print("Run a sweep with --apply first:")
        print("  python3 -m sweep.runner --sweep gait --scenario flat_walk --apply")
        sys.exit(1)

    if args.all:
        directions = load_all_directions(args.config)
        if not directions:
            print("No per-direction configs found.")
            return
        for dir_name in DIRECTIONS:
            if dir_name in directions:
                d = directions[dir_name]
                params_str = ", ".join(
                    f"{k}={v:.4f}" for k, v in d.items()
                    if not k.startswith("_")
                )
                score = d.get("_score", "?")
                print(f"  {dir_name}: {params_str} (score: {score})")
            else:
                print(f"  {dir_name}: (not tuned)")
        return

    if not os.path.exists(args.firmware_config):
        print(f"Firmware config not found at {args.firmware_config}")
        sys.exit(1)

    params = load_config(args.config, args.direction)
    gait_params = {k: v for k, v in params.items() if k in PARAM_TO_DEFINE}
    print(f"Config: {args.config}")
    print(f"Direction: {args.direction}")
    print(f"Params: {json.dumps({k: round(v, 4) for k, v in gait_params.items()})}")
    print()

    with open(args.firmware_config) as f:
        config_h = f.read()

    new_text, changes = patch_config_h(config_h, gait_params)

    if not changes:
        print("No matching parameters to update in config.h")
        return

    print("Changes:")
    for c in changes:
        print(c)
    print()

    if args.write:
        with open(args.firmware_config, "w") as f:
            f.write(new_text)
        print(f"Written to {args.firmware_config}")
        print("Run `pio run` to rebuild firmware with new parameters.")
    else:
        print("Preview only — run with --write to apply changes.")


if __name__ == "__main__":
    main()
