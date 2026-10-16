#!/usr/bin/env bash
set -eo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$root_dir/artifacts/firmware"
"${ARDUINO_CLI:-arduino-cli}" compile --fqbn arduino:avr:mega --warnings all \
  --output-dir "$root_dir/artifacts/firmware" "$root_dir/firmware/minesweeper_mega" \
  2>&1 | tee "$root_dir/artifacts/firmware_build.log"
