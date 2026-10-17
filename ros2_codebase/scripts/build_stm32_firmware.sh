#!/usr/bin/env bash
set -eo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
firmware_dir="$root_dir/firmware/stm32f411"
profile="${1:-commissioning}"
case "$profile" in
  commissioning) output_dir=Debug ;;
  mdd10a) output_dir=Debug-mdd10a ;;
  *) echo 'Usage: bash scripts/build_stm32_firmware.sh [commissioning|mdd10a]' >&2; exit 2 ;;
esac
python3 "$firmware_dir/scripts/fetch_dependencies.py"
make -C "$firmware_dir" -j"${BUILD_JOBS:-4}" MOTOR_PROFILE="$profile"
python3 "$firmware_dir/scripts/inspect_image.py" --image "$firmware_dir/$output_dir/Minesweeper_STM32F411.bin"
