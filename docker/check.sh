#!/usr/bin/env bash
# Compile-check the workspace inside the ctr-jazzy-build container.
#
# The host workspace is mounted read-only and rsynced to the named volume
# ctr_ws_src (mounted at /ws) before building — lib_robot's POST_BUILD dcfgen
# step writes master.dcf/.bin into the source tree, which a :ro mount forbids.
# rsync preserves mtimes, so together with the build volume ctr_ws_build
# incremental rebuilds stay fast, and the host tree is never written to.
#
# Usage:
#   docker/check.sh                        # full colcon build (Release)
#   docker/check.sh --packages-select mpc  # any extra args are passed to colcon build
set -euo pipefail
WS="$(cd "$(dirname "$0")/.." && pwd)"

exec docker run --rm \
  -v "$WS":/ws_ro:ro \
  -v ctr_ws_src:/ws \
  -v ctr_ws_build:/out \
  ctr-jazzy-build bash -lc "
    rsync -a --delete --exclude=.git /ws_ro/ /ws/ &&
    source /opt/ros/jazzy/setup.bash &&
    cd /out &&
    colcon build --base-paths /ws/src \
      --build-base /out/build --install-base /out/install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      $*"
