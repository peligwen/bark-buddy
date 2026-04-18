#!/usr/bin/env bash
# Claude Code PreToolUse hook — runs compile checks before Claude commits.
#
# Triggered by .claude/settings.json on any Bash call matching "git commit*".
# Checks (in order, stops at first failure):
#   1. Arduino macro collision lint  (firmware headers/sources)
#   2. Mock firmware build           (make bark-mock, ~5s)
#   3. Python syntax                 (compileall host/ bark_cli.py)
#   4. Ruff lint                     (if ruff is installed)
#
# Exits 0 → allow commit.  Exits 2 → block with JSON reason shown to Claude.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"

# Emit a Claude Code "block" decision and exit 2.
block() {
  local reason="$1"
  jq -n --arg r "$reason" '{"decision":"block","reason":$r}'
  exit 2
}

# --- 1. Arduino macro collision lint ---
echo "[pre-commit] Arduino macro collision lint..." >&2
lint_out=$("$SCRIPT_DIR/check-arduino-macro-collisions.sh" 2>&1)
lint_exit=$?
if [[ $lint_exit -ne 0 ]]; then
  block "Arduino macro collision in firmware headers — commit blocked.

$lint_out

Rename the C++ identifier so it doesn't match an Arduino preprocessor macro."
fi

# --- 2. Mock firmware build ---
echo "[pre-commit] Building mock firmware..." >&2
make_out=$(make -C "$REPO_ROOT/firmware/test" bark-mock 2>&1)
make_exit=$?
if [[ $make_exit -ne 0 ]]; then
  block "Mock firmware build failed — commit blocked.

$make_out

Fix the compile errors above, then retry."
fi

# --- 3. Python syntax check ---
echo "[pre-commit] Python syntax check..." >&2
py_out=$(python3 -m compileall -q "$REPO_ROOT/host/" "$REPO_ROOT/bark_cli.py" 2>&1)
py_exit=$?
if [[ $py_exit -ne 0 ]]; then
  block "Python syntax error — commit blocked.

$py_out"
fi

# --- 4. Ruff lint (optional: skip if not installed) ---
if python3 -m ruff --version &>/dev/null 2>&1; then
  echo "[pre-commit] Ruff lint..." >&2
  ruff_out=$(python3 -m ruff check "$REPO_ROOT/host/" "$REPO_ROOT/bark_cli.py" 2>&1)
  ruff_exit=$?
  if [[ $ruff_exit -ne 0 ]]; then
    block "Ruff lint errors — commit blocked.

$ruff_out

Run 'python3 -m ruff check --fix host/ bark_cli.py' to auto-fix what's fixable."
  fi
else
  echo "[pre-commit] ruff not installed — skipping lint (install with: pip install ruff)" >&2
fi

echo "[pre-commit] All checks passed." >&2
echo "[pre-commit] Note: ESP32 macro collisions and PlatformIO-specific errors are NOT checked here — run 'pio run' before flashing." >&2
exit 0
