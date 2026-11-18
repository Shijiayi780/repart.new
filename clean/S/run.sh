#!/usr/bin/env bash
# Usage: run.sh <input-dir> <output-file>
# S mode: default theta unless THETA=0 is set; FIRST_ONLY defaults to 1.
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
input=$(realpath "$1")
output=$(realpath -m "$2")
ulimit -v "${MEM_KB:-6291456}"
export REPART_GLOBAL_ASSIGNMENT=1
export REPART_GLOBAL_BOARD_ORDER=1
export REPART_BOARD_SCHEDULE=1
export REPART_ASSIGNMENT_FIRST_ONLY="${FIRST_ONLY:-1}"
if [ -n "${THETA:-}" ]; then export REPART_GLOBAL_THETA="$THETA"; fi
"$here/build/partitioner_S" -t "$input" -s "$output"
