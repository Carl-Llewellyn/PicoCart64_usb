#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
IMAGE_TAG=${IMAGE_TAG:-picocart64-ubuntu-2024}
PICO_SDK_REF=${PICO_SDK_REF:-2.1.1}

exec docker build \
  --build-arg PICO_SDK_REF="${PICO_SDK_REF}" \
  -t "${IMAGE_TAG}" \
  -f "${SCRIPT_DIR}/Dockerfile" \
  "${SCRIPT_DIR}"
