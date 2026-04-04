#!/usr/bin/env python3
"""
Apply gait_config.json parameters to firmware/include/config.h.

Reads the tuned parameters from gait_config.json (produced by sweep runner
with --apply) and patches the corresponding #define values in config.h.

Usage:
    python3 apply_to_firmware.py              # preview changes
    python3 apply_to_firmware.py --write      # write changes to config.h
    python3 apply_to_firmware.py --config path/to/config.json  # custom config
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


def load_config(path: str) -> dict:
    with open(path) as f:
        data = json.load(f)
    return data.get("params", data)


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

        # Match: #define GAIT_HIP_AMPLITUDE   8.0f
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
    parser.add_argument("--write", action="store_true",
                        help="Write changes (default: preview only)")
    args = parser.parse_args()

    if not os.path.exists(args.config):
        print(f"No config file at {args.config}")
        print("Run a sweep with --apply first:")
        print("  python3 -m sweep.runner --sweep gait --scenario flat_walk --apply")
        sys.exit(1)

    if not os.path.exists(args.firmware_config):
        print(f"Firmware config not found at {args.firmware_config}")
        sys.exit(1)

    params = load_config(args.config)
    print(f"Config: {args.config}")
    print(f"Params: {json.dumps({k: round(v, 4) for k, v in params.items()})}")
    print()

    with open(args.firmware_config) as f:
        config_h = f.read()

    new_text, changes = patch_config_h(config_h, params)

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
