#!/bin/bash
# Host-side compile check of the E1 Zenoh link against the real zenoh-pico API (no ESP32 toolchain needed).
#   firmware/test/host_check/run.sh [zenoh-pico tag]
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); FW=$(cd "$HERE/../.." && pwd)
TAG=${1:-1.10.1}; ZP=${ZP_DIR:-/tmp/zenoh-pico-$TAG}
if [ ! -d "$ZP/include" ]; then                # retry: GitHub occasionally answers 403/5xx to CI clones
  for i in 1 2 3 4 5; do
    rm -rf "$ZP"; git clone -q --depth 1 -b "$TAG" https://github.com/eclipse-zenoh/zenoh-pico "$ZP" && break
    [ "$i" = 5 ] && exit 1; echo "clone failed, retrying in $((i * 10)) s"; sleep $((i * 10))
  done
fi
[ -f "$ZP/build/include/zenoh-pico/config.h" ] || cmake -S "$ZP" -B "$ZP/build" -DBUILD_SHARED_LIBS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF >/dev/null
g++ -std=gnu++17 -fsyntax-only -Wall -Wno-unused-variable -Wno-reorder -DZENOH_LINUX -DZENOH_COMPILER_GCC \
    -I"$HERE" -I"$FW/src_e1" -I"$FW/external/CF-Drone" -I"$ZP/build/include" -I"$ZP/include" "$HERE/check.cpp"
echo "host check OK (zenoh-pico $TAG)"
