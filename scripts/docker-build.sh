#!/usr/bin/env bash
set -euo pipefail

IMAGE="${IMAGE:-cms32foc-toolchain:ubuntu24.04}"
BUILD_DIR="${BUILD_DIR:-build/docker-gcc-debug}"

docker run --rm -t \
    -u "$(id -u):$(id -g)" \
    -e BUILD_DIR="$BUILD_DIR" \
    -v "$PWD":/workspace \
    -w /workspace \
    "$IMAGE" \
    bash -lc 'cmake -S . -B "$BUILD_DIR" -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake -DCMAKE_BUILD_TYPE=Debug && cmake --build "$BUILD_DIR"'
