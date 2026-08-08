# Docker build-verification environment

The development machine has no ROS 2, so every change to this workspace is compile-checked
inside a `ros:jazzy` container that mirrors the lab machine's hand-installed `/usr/local`
dependency layout (LibTorch at `/usr/local/libtorch`, OMPL config at
`/usr/local/share/ompl/cmake`, OpenIGTLink at `/usr/local/lib/igtl/cmake/igtl-3.1`).

```bash
docker/build.sh      # build the image (one-time; ~15-30 min, downloads LibTorch and compiles OMPL)
docker/check.sh      # full colcon build of the workspace (Release), incremental via volume ctr_ws_build
docker/check.sh --packages-select mpc     # narrower check; extra args go to colcon build
docker volume rm ctr_ws_build             # clean build state
```

Binaries produced here are check-only — deployment builds still happen on the lab machine.

## Pinned versions (confirm against the lab machine once)

| Dependency | Image version | Notes |
|---|---|---|
| LibTorch | 2.9.0 CPU | matches the `Torch_DIR` pin in `ctr_pinn_infer`'s CMake |
| OMPL | 1.6.0 | source build |
| OpenIGTLink | 3.1 | source build |
| OSQP / OsqpEigen | 0.6.3 / 0.8.1 | source builds |
| lely-core (+dcfgen) | master / PyPI `dcf-tools` | dcfgen runs at build time on `master.yml` |
| Blaze, Boost, NLopt, TBB, FCL, Eigen, spdlog, yaml-cpp, Qt5 | Ubuntu 24.04 apt | |
