#!/usr/bin/env bash
# Build the ctr-jazzy-build verification image. One-time (rebuild only when the Dockerfile changes).
set -euo pipefail
cd "$(dirname "$0")"
exec docker build -t ctr-jazzy-build .
