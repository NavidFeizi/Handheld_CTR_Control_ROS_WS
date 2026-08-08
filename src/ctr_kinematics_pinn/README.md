# ctr_kinematics_pinn

ROS-free TorchScript PINN inference for the handheld CTR — forward kinematics,
Jacobians (incl. w.r.t. tip force), and resolved-rate inverse kinematics
(`posCTRL`) over Blaze types. This package is the merger of the formerly
duplicated `robot/ctr_pinn_infer` and `mpc/ctr_pinn_infer` copies (byte-identical)
and the planner's diverged `PINNs.hpp` fork, plus the unified `models/` pool.

## Usage

```cmake
find_package(ctr_kinematics_pinn REQUIRED)   # also runs the LibTorch pin
target_link_libraries(my_target ctr_kinematics_pinn::ctr_kinematics_pinn)
```

```cpp
#include "ctr_kinematics_pinn/ctr_pinn_inference.hpp"
PINNs<4> pinn(models_dir, model_name, batch, backbone_nodes);
```

`models_dir` is usually resolved via `ctr_common::resolveModelsDir(node)`:
node parameter `models_dir` if set, else this package's installed
`share/ctr_kinematics_pinn/models`.

## LibTorch pin

`cmake/torch_pin.cmake` pins Torch to the self-contained install at
`/usr/local/libtorch` (override with `-DCTR_TORCH_DIR=<prefix>`) and works
around stale-header/ABI conflicts with any system PyTorch — see the comments
in that file. It runs both when this package builds and, via `CONFIG_EXTRAS`,
whenever a downstream package calls `find_package(ctr_kinematics_pinn)`.

## Models

Each `models/<name>/` directory holds exactly `model_scripted.pt` (TorchScript
module) and `parameters.json` (physics/model/dataset parameters). Training
artifacts stay out of the repo.
