#!/usr/bin/env bash
# H variant: community-guided search; shares clean/baseline source.
set -euo pipefail
cd "$(dirname "$0")"
root=$(pwd)
src="$root/../baseline/RePart"
boost="$root/../third_party/boost_1_86_0"
mkdir -p build
g++ -std=c++17 -O2 -DNDEBUG -fno-lto -DREPART_CF_OBSERVE \
  -c "$src/community/detector_bridge.cpp" -o build/detector.o \
  > build/detector.log 2>&1
g++ -std=c++17 -Ofast -DNDEBUG -DREPART_CF_OBSERVE -DREPART_COMMUNITY_SEARCH \
  -I"$boost/include" "$src/partitioner.cpp" build/detector.o \
  -L"$boost/lib" -static -lboost_thread -lboost_system -pthread \
  -o build/partitioner_H > build/build.log 2>&1
echo "built $(pwd)/build/partitioner_H"
