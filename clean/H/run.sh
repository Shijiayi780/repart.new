#!/usr/bin/env bash
# Usage: run.sh <input-dir> <output-file>
# H mode; override DELAY (default 0) if needed.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
input=$(realpath "$1")
output=$(realpath -m "$2")
ulimit -v "${MEM_KB:-6291456}"
REPART_SEARCH_MODE=H REPART_SEARCH_DELAY_LEVELS="${DELAY:-0}" \
  "$here/build/partitioner_H" -t "$input" -s "$output"
