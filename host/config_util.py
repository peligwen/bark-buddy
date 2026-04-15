"""Utilities for reading values from firmware/include/config_local.h."""

import os
import re


def read_config_local_value(key: str, default: str | None) -> str | None:
    """Read a #define string value from firmware/include/config_local.h.

    Uses a line-anchored regex to avoid matching values inside // comments.
    """
    config_path = os.path.join(os.path.dirname(__file__), "..", "firmware", "include", "config_local.h")
    try:
        with open(config_path) as f:
            content = f.read()
        m = re.search(r'^[ \t]*#define\s+' + re.escape(key) + r'\s+"([^"]+)"', content, re.MULTILINE)
        return m.group(1) if m else default
    except FileNotFoundError:
        return default
