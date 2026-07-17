# CTR PINN Inference Library

A C++ library for inference with Physics-Informed Neural Networks for Concentric Tube Robots. It serves as a deployment package for models trained using Python packages, offering forward passes through the DNN for:
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

navigate to the repositry root

```sh
cmake -S . -B build
cmake --build build
cmake --install build --prefix ~./local
```
you may use any other path instead of `~./local`

## Usage

see under examples/cpp/pinn_inference.cpp for examples
