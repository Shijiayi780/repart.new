#!/usr/bin/env bash
# Clean A baseline: no experiment macros.
set -euo pipefail
cd "$(dirname "$0")"
root=$(pwd)
boost="$root/../third_party/boost_1_86_0"
mkdir -p build
g++ -std=c++17 -Ofast -DNDEBUG -I"$boost/include" RePart/partitioner.cpp \
  -L"$boost/lib" -static -lboost_thread -lboost_system -pthread \
  -o build/partitioner_A > build/build.log 2>&1
echo "built $(pwd)/build/partitioner_A"
