# CTR PINN Inference Library

A C++ library for inference with PINN for CTR. It serves as a deployment package for models trained using Python packages, offering forward passes through the PINN for:
- distal end position of the CTR: get_pos_distal()
- distal end position of each tube: get_pos_tubes()
- backbone shape for s[0-L1]: get_shape()
- complete states for s[0-L1]: get_entire_state()


## Requirements

* [LibTorch](https://pytorch.org/get-started/locally/) - C++ PyTorch package (CPU version)

    After installation, you may need to add the following to your `.bashrc` file:

    ```bash
    export Torch_DIR="/usr/local/libtorch/share/cmake/Torch"
    ```

* [Blaze Library](https://bitbucket.org/blaze-lib/blaze/src/master/) – Header-only linear algebra

* [libnlohmann-json](https://github.com/nlohmann/json)
    ```bash
    sudo apt install -y nlohmann-json3-dev
    ```
## Installation
`
Navigate to the repository root:

```sh
cmake -S . -B build -DBUILD_PINN_INFER=ON
cmake --build build -j
cmake --install build --prefix ~/.local
```
You may use any other prefix instead of `~/.local`.

## Usage

See `examples/pinn_infer.cpp` for a C++ usage example.
