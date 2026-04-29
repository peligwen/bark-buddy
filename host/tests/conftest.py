"""pytest config — make host/ importable from tests."""
import os
import sys

_HOST_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _HOST_DIR not in sys.path:
    sys.path.insert(0, _HOST_DIR)
