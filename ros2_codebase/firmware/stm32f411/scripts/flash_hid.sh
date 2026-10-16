#!/usr/bin/env bash
set -euo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" != --motor-power-isolated ]]; then
  echo 'Physically disconnect motor power, then use: bash scripts/flash_hid.sh --motor-power-isolated' >&2
  exit 2
fi
firmware="$root_dir/Debug/Minesweeper_STM32F411.bin"
test -s "$firmware"
python3 "$root_dir/scripts/inspect_image.py"
python3 - <<'PY'
from pathlib import Path
import os
devices=[]
for vendor in Path('/sys/bus/usb/devices').glob('*/idVendor'):
    dev=vendor.parent
    if vendor.read_text().strip()=='0483' and (dev/'idProduct').read_text().strip()=='572a':
        devices.append(dev)
if len(devices)!=1:
    raise SystemExit('Expected exactly one WeAct HID bootloader (0483:572a).')
dev=devices[0]
path=f"/dev/bus/usb/{int((dev/'busnum').read_text()):03d}/{int((dev/'devnum').read_text()):03d}"
try:
    os.close(os.open(path,os.O_RDWR))
except PermissionError:
    raise SystemExit(f'USB write access denied: {path}. Apply docs/FLASHING.md device permissions first.')
PY
mkdir -p "$root_dir/.cache/tools"
tool="$root_dir/.cache/tools/WeAct_HID_Flash-CLI"
if [[ ! -f "$tool" ]]; then
  curl --fail --location --max-time 120 \
    'https://raw.githubusercontent.com/WeActStudio/WeActStudio.MiniSTM32F4x1/master/Soft/WeAct_HID_FW_Bootloader/WeAct_HID_Flash-CLI' \
    --output "$tool"
fi
echo "cd8d6b53f5d43f9c8aaaeefaf7fdc34593de9636b5d2d2652294eb48f79aec74  $tool" | sha256sum --check
chmod u+x "$tool"
# Vendor tool uses the existing bootloader's 0x08004000 application region.
# Do not use its erase command or a full-flash image.
"$tool" "$firmware" reboot | tee "$root_dir/Debug/flash_hid.log"
if ! rg -q 'Flash Done !' "$root_dir/Debug/flash_hid.log"; then
  echo 'Vendor tool did not report success; flashing is NOT verified.' >&2
  exit 1
fi
echo 'Bootloader reports programming complete. Verify USB enumeration and /motor/status; see docs/VALIDATION.md.'
