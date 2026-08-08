# Build-verification image for the Handheld CTR workspace.
# Mirrors the lab machine's dependency layout (everything hand-installed at /usr/local)
# so the workspace's hardcoded find paths resolve identically. Used by check.sh for
# compile checks only — binaries built here are NOT deployed.
FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget unzip pkg-config ca-certificates \
    autoconf automake libtool \
    libboost-all-dev \
    libblas-dev liblapack-dev libopenblas-dev gfortran \
    libtbb-dev \
    libnlopt-cxx-dev \
    libfcl-dev libassimp-dev libpugixml-dev \
    nlohmann-json3-dev libspdlog-dev libeigen3-dev \
    libyaml-cpp-dev \
    qtbase5-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Blaze (header-only). Prefer apt; fall back to source (same 3.8.x line).
RUN apt-get update && (apt-get install -y --no-install-recommends libblaze-dev \
    || (git clone --depth 1 --branch v3.8.2 https://bitbucket.org/blaze-lib/blaze.git /tmp/blaze \
        && cmake -S /tmp/blaze -B /tmp/blaze/build -DCMAKE_BUILD_TYPE=Release \
        && cmake --install /tmp/blaze/build && rm -rf /tmp/blaze)) \
    && rm -rf /var/lib/apt/lists/*

# LibTorch 2.9.0 CPU at the exact lab path /usr/local/libtorch (matches the
# Torch_DIR pin the planner relies on). Try the cxx11-abi artifact name first,
# fall back to the newer unified name.
RUN (wget -qO /tmp/libtorch.zip "https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.9.0%2Bcpu.zip" \
     || wget -qO /tmp/libtorch.zip "https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-2.9.0%2Bcpu.zip") \
    && unzip -q /tmp/libtorch.zip -d /usr/local && rm /tmp/libtorch.zip

# OMPL 1.6.0 from source → /usr/local (config lands at /usr/local/share/ompl/cmake,
# the path hardcoded in planner/motion_planning/CMakeLists.txt).
RUN git clone --depth 1 --branch 1.6.0 https://github.com/ompl/ompl.git /tmp/ompl \
    && cmake -S /tmp/ompl -B /tmp/ompl/build -DCMAKE_BUILD_TYPE=Release \
         -DOMPL_BUILD_PYBINDINGS=OFF -DOMPL_BUILD_DEMOS=OFF -DOMPL_BUILD_TESTS=OFF -DOMPL_REGISTRATION=OFF \
    && cmake --build /tmp/ompl/build -j"$(nproc)" \
    && cmake --install /tmp/ompl/build && rm -rf /tmp/ompl

# OpenIGTLink 3.1 from source → /usr/local (config lands at /usr/local/lib/igtl/cmake/igtl-3.1,
# the path hardcoded in emtracker/ and igtlink_bridge/). No 3.1 tag was ever cut upstream —
# master carries version 3.1 (the lab install can only have come from there), so build master.
RUN git clone --depth 1 https://github.com/openigtlink/OpenIGTLink.git /tmp/igtl \
    && cmake -S /tmp/igtl -B /tmp/igtl/build -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    && cmake --build /tmp/igtl/build -j"$(nproc)" \
    && cmake --install /tmp/igtl/build && rm -rf /tmp/igtl

# OSQP 0.6.3 + OsqpEigen 0.8.1 from source (no apt packages; mpc needs both).
RUN git clone --depth 1 --branch v0.6.3 --recursive https://github.com/osqp/osqp.git /tmp/osqp \
    && cmake -S /tmp/osqp -B /tmp/osqp/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/osqp/build -j"$(nproc)" \
    && cmake --install /tmp/osqp/build && rm -rf /tmp/osqp
RUN git clone --depth 1 --branch v0.8.1 https://github.com/robotology/osqp-eigen.git /tmp/osqpe \
    && cmake -S /tmp/osqpe -B /tmp/osqpe/build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && cmake --build /tmp/osqpe/build -j"$(nproc)" \
    && cmake --install /tmp/osqpe/build && rm -rf /tmp/osqpe

# lely-core (liblely-coapp, CANopen) from source; dcfgen from PyPI (dcf-tools).
RUN git clone --depth 1 https://gitlab.com/lely_industries/lely-core.git /tmp/lely \
    && cd /tmp/lely && autoreconf -i \
    && ./configure --disable-python --disable-tests --disable-doc \
    && make -j"$(nproc)" && make install && ldconfig && rm -rf /tmp/lely
RUN pip install --break-system-packages --no-cache-dir dcf-tools

# The robot driver reads $HOME-based paths at runtime; harmless at build time.
WORKDIR /out
