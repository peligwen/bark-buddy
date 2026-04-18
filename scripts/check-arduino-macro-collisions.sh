#!/usr/bin/env bash
# Lint firmware headers and sources for C++ identifiers that collide with
# Arduino macro names (PULLUP, PULLDOWN, OUTPUT, etc.). These collisions cause
# silent preprocessor corruption that only manifests in the real ESP32 build,
# not in the mock/native build.
#
# Usage: scripts/check-arduino-macro-collisions.sh [--quiet]
#        Returns exit code 0 if clean, 1 on any collision found.
#
# Silence a specific line with:  // NOLINT(arduino-macro)

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)"

QUIET=0
[[ "${1:-}" == "--quiet" ]] && QUIET=1

# Arduino macros defined in esp32-hal-gpio.h and related headers that are
# short/generic enough to collide accidentally with C++ identifiers.
MACROS='PULLUP|PULLDOWN|OUTPUT|INPUT|HIGH|LOW|ANALOG|INPUT_PULLUP|INPUT_PULLDOWN|RISING|FALLING|CHANGE'

SEARCH_DIRS=(
  "$REPO_ROOT/firmware/include"
  "$REPO_ROOT/firmware/src"
)

errors=0

for dir in "${SEARCH_DIRS[@]}"; do
  [[ -d "$dir" ]] || continue
  while IFS= read -r -d '' file; do
    while IFS=: read -r lineno content; do
      [[ -z "${content// }" ]] && continue

      # Strip leading whitespace for prefix checks
      trimmed="${content#"${content%%[! $'\t']*}"}"

      # Skip pure comment lines
      [[ "$trimmed" == //* ]] && continue
      [[ "$trimmed" == "/*"* ]] && continue
      [[ "$trimmed" == "* "* ]] && continue

      # Skip all preprocessor lines (#define, #include, #ifdef, etc.)
      [[ "$trimmed" == "#"* ]] && continue

      # Skip NOLINT suppression
      [[ "$content" == *"NOLINT(arduino-macro)"* ]] && continue

      # Strip trailing single-line comment before re-checking: if the macro
      # only appears inside a // comment, it's not a C++ identifier declaration.
      code_part="${content%%//*}"
      echo "$code_part" | grep -qE "[[:<:]]($MACROS)[[:>:]]" || continue

      # Skip lines that are clearly calling Arduino GPIO/interrupt APIs
      # (these are legitimate macro uses, not C++ identifier declarations)
      for api in pinMode digitalWrite digitalRead analogRead analogWrite \
                 attachInterrupt detachInterrupt ledcSetup ledcWrite; do
        [[ "$content" == *"${api}("* ]] && continue 2
      done

      [[ $QUIET -eq 0 ]] && echo "$file:$lineno: arduino-macro: $content"
      errors=$((errors + 1))

    done < <(grep -nE "[[:<:]]($MACROS)[[:>:]]" "$file" 2>/dev/null || true)
  done < <(find "$dir" \( -name "*.h" -o -name "*.cpp" \) -print0 2>/dev/null)
done

if [[ $errors -gt 0 ]]; then
  echo ""
  echo "arduino-macro lint: $errors collision(s) found."
  echo "Rename the C++ identifier or add  // NOLINT(arduino-macro)  to suppress."
  exit 1
fi

[[ $QUIET -eq 0 ]] && echo "arduino-macro lint: OK"
exit 0
