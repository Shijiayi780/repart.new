#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
bash baseline/build.sh
bash H/build.sh
bash S/build.sh
echo "ALL_BUILDS_OK"
