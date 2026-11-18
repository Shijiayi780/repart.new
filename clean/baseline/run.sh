#!/usr/bin/env bash
# Usage: run.sh <input-dir> <output-file>
set -euo pipefail
here=$(cd "$(dirname "$0")" && pwd)
input=$(realpath "$1")
output=$(realpath -m "$2")
ulimit -v "${MEM_KB:-6291456}"
"$here/build/partitioner_A" -t "$input" -s "$output"
