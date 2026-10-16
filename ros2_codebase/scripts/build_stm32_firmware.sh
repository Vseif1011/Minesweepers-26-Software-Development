#!/usr/bin/env bash
set -eo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
firmware_dir="$root_dir/firmware/stm32f411"
python3 "$firmware_dir/scripts/fetch_dependencies.py"
make -C "$firmware_dir" -j"${BUILD_JOBS:-4}"
python3 "$firmware_dir/scripts/inspect_image.py"
