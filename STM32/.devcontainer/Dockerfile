FROM ubuntu:24.04

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

# Same toolchain as .github/workflows/stm32-build.yml (gcc-arm-none-eabi + ninja-build + cmake).
# Keep this in sync with that workflow.
# gdb-multiarch / openocd / stlink-tools are added on top for optional
# in-container flashing and debugging (ST-Link over USB passthrough).
RUN apt-get update -y && apt-get install -y --no-install-recommends \
        git \
        ca-certificates \
        cmake \
        ninja-build \
        gcc-arm-none-eabi \
        gdb-multiarch \
        openocd \
        stlink-tools \
        udev \
    && rm -rf /var/lib/apt/lists/*
