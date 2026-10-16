#!/usr/bin/env python3
"""Compare native ROS 2 and embedded generated interface type hashes."""
import argparse
from pathlib import Path
import re
parser = argparse.ArgumentParser()
parser.add_argument('--native-build', type=Path, required=True,
                    help='Native colcon build/minesweeper_interfaces directory')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
relative = Path('rosidl_generator_c/minesweeper_interfaces/msg/detail')
cross = root / '.cache/client_ws/build/minesweeper_interfaces' / relative
native = args.native_build / relative
for name in ['motor_command', 'motor_status', 'encoder_ticks']:
    filename = name + '__description.c'
    values = []
    for base in [cross, native]:
        text = (base / filename).read_text()
        body = re.search(r'static rosidl_type_hash_t hash\s*=\s*\{1,\s*\{([^}]+)', text)
        if body is None:
            raise SystemExit(f'Missing generated type hash in {base / filename}')
        values.append(bytes(int(value, 16) for value in re.findall(r'0x([0-9a-fA-F]{2})', body[1])))
    assert values[0] == values[1] and len(values[0]) == 32, f'{name}: native/embedded mismatch'
    print(f'{name}: native/embedded type hash matches ({values[0].hex()})')
