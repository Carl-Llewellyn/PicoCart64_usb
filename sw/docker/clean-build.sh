#!/usr/bin/env bash
set -euo pipefail

NO_ROM_COMPRESS=0
POSITIONAL=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-rom-compress)
      NO_ROM_COMPRESS=1
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [--no-rom-compress] /absolute/path/to/rom.z64"
      exit 0
      ;;
    -*)
      echo "Unknown option: $1" >&2
      echo "Usage: $0 [--no-rom-compress] /absolute/path/to/rom.z64" >&2
      exit 1
      ;;
    *)
      POSITIONAL+=("$1")
      shift
      ;;
  esac
done

set -- "${POSITIONAL[@]}"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 [--no-rom-compress] /absolute/path/to/rom.z64"
  exit 1
fi

ROM_PATH=$(realpath "$1")
if [[ ! -f "${ROM_PATH}" ]]; then
  echo "ROM not found: ${ROM_PATH}" >&2
  exit 1
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SW_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
REPO_ROOT=$(git -C "${SW_DIR}" rev-parse --show-toplevel)
SW_RELATIVE=$(realpath --relative-to "${REPO_ROOT}" "${SW_DIR}")
CONTAINER_SW_DIR="/repo/${SW_RELATIVE}"
IMAGE_TAG=${IMAGE_TAG:-picocart64-ubuntu-2024}
REGION=${REGION:-NTSC}
FLASH_SIZE_MB=${FLASH_SIZE_MB:-16}
JOBS=${JOBS:-$(nproc)}

FREERTOS_KERNEL_REF=${FREERTOS_KERNEL_REF:-V11.1.0}
FORCE_FREERTOS_REF=${FORCE_FREERTOS_REF:-0}

LOAD_ROM_COMPRESS_ARG="--compress"
if [[ "${NO_ROM_COMPRESS}" == "1" ]]; then
  LOAD_ROM_COMPRESS_ARG=""
fi

if [[ ! -d "${SW_DIR}/lib/freertos-kernel" || -z "$(ls -A "${SW_DIR}/lib/freertos-kernel" 2>/dev/null)" ]]; then
  echo "FreeRTOS kernel missing; initializing in ${SW_DIR}/lib/freertos-kernel" >&2
  rm -rf "${SW_DIR}/lib/freertos-kernel"
  git -C "${SW_DIR}" submodule update --init lib/freertos-kernel || true
  if [[ ! -d "${SW_DIR}/lib/freertos-kernel" || -z "$(ls -A "${SW_DIR}/lib/freertos-kernel" 2>/dev/null)" ]]; then
    git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git "${SW_DIR}/lib/freertos-kernel"
    git -C "${SW_DIR}/lib/freertos-kernel" checkout "${FREERTOS_KERNEL_REF}"
  fi
elif [[ "${FORCE_FREERTOS_REF}" == "1" ]]; then
  git -C "${SW_DIR}/lib/freertos-kernel" fetch --tags
  git -C "${SW_DIR}/lib/freertos-kernel" checkout "${FREERTOS_KERNEL_REF}"
fi

docker run --rm \
  -u "$(id -u):$(id -g)" \
  -v "${REPO_ROOT}":/repo \
  -v "${ROM_PATH}":/rom.z64:ro \
  -e PICO_SDK_PATH=/opt/pico-sdk \
  "${IMAGE_TAG}" \
  bash -lc "rm -rf ${CONTAINER_SW_DIR}/build ${CONTAINER_SW_DIR}/CMakeCache.txt ${CONTAINER_SW_DIR}/CMakeFiles \
    && python3 ${CONTAINER_SW_DIR}/scripts/load_rom.py ${LOAD_ROM_COMPRESS_ARG} /rom.z64 \
    && mkdir ${CONTAINER_SW_DIR}/build \
    && cd ${CONTAINER_SW_DIR}/build \
    && cmake -DPICO_SDK_PATH=/opt/pico-sdk -DREGION=${REGION} -DFLASH_SIZE_MB=${FLASH_SIZE_MB} -DPICO_FLASH_SIZE_BYTES=16777216 .. \
    && cmake --build ${CONTAINER_SW_DIR}/build --target picocart64_v1 -j ${JOBS}"
