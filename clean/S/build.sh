#!/usr/bin/env bash
# S variant: global assignment scoring; board order and schedule are runtime switches.
set -euo pipefail
cd "$(dirname "$0")"
root=$(pwd)
src="$root/../baseline/RePart"
boost="$root/../third_party/boost_1_86_0"
mkdir -p build
g++ -std=c++17 -Ofast -DNDEBUG -DREPART_GLOBAL_ASSIGNMENT \
  -I"$boost/include" "$src/partitioner.cpp" \
  -L"$boost/lib" -static -lboost_thread -lboost_system -pthread \
  -o build/partitioner_S > build/build.log 2>&1
echo "built $(pwd)/build/partitioner_S"
