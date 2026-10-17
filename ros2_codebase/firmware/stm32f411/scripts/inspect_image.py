#!/usr/bin/env python3
"""Verify flash bounds and vector table before downloading the existing image."""
from pathlib import Path
import hashlib
import struct
import argparse
root = Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--image',type=Path,default=root/'Debug/Minesweeper_STM32F411.bin')
image = parser.parse_args().image
data = image.read_bytes()
stack, reset = struct.unpack_from('<II', data)
assert 0 < len(data) <= 496*1024, 'Application exceeds reserved flash region'
assert 0x20000000 < stack <= 0x20020000 and stack % 8 == 0, 'Bad initial stack'
assert reset & 1 and 0x08004000 <= (reset & ~1) < 0x08004000+len(data), 'Bad reset vector/origin'
print(f'Image: {len(data)} bytes, SP={stack:#010x}, reset={reset:#010x}, origin=0x08004000')
print('SHA256:', hashlib.sha256(data).hexdigest())
