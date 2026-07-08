#!/usr/bin/env bash
set -euo pipefail

IMAGE="${IMAGE:-cms32foc-toolchain:ubuntu24.04}"
OUT="${OUT:-cms32foc-toolchain-ubuntu24.04.tar}"

docker save -o "$OUT" "$IMAGE"
echo "Wrote $OUT"
