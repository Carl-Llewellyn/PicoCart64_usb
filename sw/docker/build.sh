#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SW_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
REPO_ROOT=$(git -C "${SW_DIR}" rev-parse --show-toplevel)
SW_RELATIVE=$(realpath --relative-to "${REPO_ROOT}" "${SW_DIR}")
CONTAINER_SW_DIR="/repo/${SW_RELATIVE}"
IMAGE_TAG=${IMAGE_TAG:-picocart64-ubuntu-2024}
JOBS=${JOBS:-$(nproc)}

docker run --rm \
  -u "$(id -u):$(id -g)" \
  -v "${REPO_ROOT}":/repo \
  -e PICO_SDK_PATH=/opt/pico-sdk \
  "${IMAGE_TAG}" \
  bash -lc "cmake --build ${CONTAINER_SW_DIR}/build --target picocart64_v1 -j ${JOBS}"
