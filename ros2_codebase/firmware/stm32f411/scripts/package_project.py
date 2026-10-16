#!/usr/bin/env python3
"""Bundle prepared CubeIDE sources, vendor code, client library and ARM artifacts."""
from pathlib import Path
import zipfile
root = Path(__file__).resolve().parents[1]
repository = root.parents[1]
out = repository / 'artifacts/stm32f411/Minesweeper_STM32F411_CubeIDE.zip'
assert (root / 'Debug/Minesweeper_STM32F411.elf').exists(), 'Build firmware first'
out.parent.mkdir(parents=True, exist_ok=True)
with zipfile.ZipFile(out, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
    for base in [root, repository / 'interfaces/minesweeper_interfaces']:
        for file in sorted(base.rglob('*')):
            relative = file.relative_to(base)
            if relative.parts[0] in {'micro_ros', 'gateway'}:
                continue
            if any(part in {'.git', '.cache', '__pycache__', 'output'} for part in relative.parts):
                continue
            if not file.is_file():
                continue
            if relative.parts[0] == 'Debug' and file.suffix not in {'.elf', '.bin', '.hex', '.map', '.log', '.json'}:
                continue
            archive.write(file, file.relative_to(repository))
print(out)
print(f'{out.stat().st_size:,} bytes; import firmware/stm32f411 after extracting')
