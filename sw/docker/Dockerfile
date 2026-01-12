FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG PICO_SDK_REF=2.1.1

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      cmake \
      gcc-arm-none-eabi \
      git \
      libnewlib-arm-none-eabi \
      libstdc++-arm-none-eabi-newlib \
      ninja-build \
      pkg-config \
      python3 \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/raspberrypi/pico-sdk.git /opt/pico-sdk \
    && cd /opt/pico-sdk \
    && git checkout "${PICO_SDK_REF}" \
    && git submodule update --init --recursive

ENV PICO_SDK_PATH=/opt/pico-sdk
WORKDIR /work
