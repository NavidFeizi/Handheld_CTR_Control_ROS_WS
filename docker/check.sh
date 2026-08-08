#!/usr/bin/env bash
# Compile-check the workspace inside the ctr-jazzy-build container.
# The workspace is mounted read-only; build/install/log artifacts live in the
# named volume ctr_ws_build so incremental rebuilds are fast.
#
# Usage:
#   docker/check.sh                        # full colcon build (Release)
#   docker/check.sh --packages-select mpc  # any extra args are passed to colcon build
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"

exec docker run --rm \
  -v "$WS":/ws:ro \
  -v ctr_ws_build:/out \
  ctr-jazzy-build bash -lc "
    source /opt/ros/jazzy/setup.bash &&
    cd /out &&
    colcon build --base-paths /ws/src \
      --build-base /out/build --install-base /out/install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      $*"
