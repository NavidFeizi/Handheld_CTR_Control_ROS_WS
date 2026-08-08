# Docker build-verification environment

The development machine has no ROS 2, so every change to this workspace is compile-checked
inside a `ros:jazzy` container that mirrors the lab machine's hand-installed `/usr/local`
dependency layout (LibTorch at `/usr/local/libtorch`, OMPL config at
`/usr/local/share/ompl/cmake`, OpenIGTLink at `/usr/local/lib/igtl/cmake/igtl-3.1`).

```bash
docker/build.sh      # build the image (one-time; ~15-30 min, downloads LibTorch and compiles OMPL)
docker/check.sh      # full colcon build of the workspace (Release), incremental via volume ctr_ws_build
docker/check.sh --packages-select mpc     # narrower check; extra args go to colcon build
docker volume rm ctr_ws_build ctr_ws_src  # clean build state
```

Binaries produced here are check-only — deployment builds still happen on the lab machine.

The host workspace is mounted read-only; check.sh rsyncs it to the `ctr_ws_src` volume
inside the container and builds from that copy (lib_robot's POST_BUILD `dcfgen` step
writes `master.dcf`/`.bin` into the source tree, as it does on the lab machine).

## Pinned versions (confirm against the lab machine once)

| Dependency | Image version | Notes |
|---|---|---|
| LibTorch | 2.9.0 CPU | matches the `Torch_DIR` pin in `ctr_pinn_infer`'s CMake |
| OMPL | 2.0.1 | source build; 2.0.x confirmed required — `Planner.hpp` uses AORRTC (first shipped in 2.0.0). Headers symlinked flat to `/usr/local/include/ompl` (lab-parity; `ament_target_dependencies` ignores the `ompl::ompl` target) |
| OpenIGTLink | master (3.1) | source build; upstream never tagged 3.1 — master carries the 3.1 version that lands at the hardcoded `igtl-3.1` config path |
| OSQP / OsqpEigen | 0.6.3 / 0.8.1 | source builds |
| lely-core (+dcfgen) | master / PyPI `dcf-tools` | dcfgen runs at build time on `master.yml` |
| Blaze, Boost, NLopt, TBB, FCL, Eigen, spdlog, yaml-cpp, Qt5 | Ubuntu 24.04 apt | |
