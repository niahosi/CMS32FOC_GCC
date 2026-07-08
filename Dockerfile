FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive

RUN printf '%s\n' \
        'deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu noble main restricted universe multiverse' \
        'deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu noble-updates main restricted universe multiverse' \
        'deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu noble-security main restricted universe multiverse' \
        > /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        build-essential \
        cmake \
        ninja-build \
        make \
        git \
        python3 \
        gcc-arm-none-eabi \
        binutils-arm-none-eabi \
        libnewlib-arm-none-eabi \
        gdb-multiarch \
        openocd \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

CMD ["/bin/bash"]
