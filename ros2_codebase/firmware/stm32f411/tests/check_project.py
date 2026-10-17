#!/usr/bin/env python3
"""Check the shipped motor-disabled profile and fixed-memory wire contracts."""
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET
root = Path(__file__).resolve().parents[1]
for name in ['.project', '.cproject']:
    ET.parse(root / name)
assert '#define BOARD_HARDWARE_APPROVED 0' in (root / 'Drivers/Board/Inc/board.h').read_text()
board = (root / 'Drivers/Board/Src/board.c').read_text()
assert board.count('MOTOR_BINDING(&htim2') == 4 and board.count('MOTOR_BINDING(&htim1') == 1
for pin in [10,12,13,14,15]:
    assert 'GPIO_PIN_'+str(pin) in board
assert '.enable_present=false' in board and 'DUTY_CONFIG(.20f)' in board
assert 'board_encoders[ENCODER_COUNT] = {0}' in board
ioc = dict(line.split('=', 1) for line in (root / 'Minesweeper_STM32F411.ioc').read_text().splitlines() if '=' in line)
assert ioc['Mcu.UserName'] == 'STM32F411CEUx'
assert ioc['RCC.PLLN'] == '192' and ioc['RCC.PLLQ'] == '4'
assert ioc['RCC.PLLM'] == '25' and ioc['RCC.HSE_VALUE'] == '25000000'
assert not any(re.match(r'P[A-H]\d+\.Signal', key) and 'GPIO_Output' in val for key, val in ioc.items())
assert all(pin + '.Signal' not in ioc for pin in ['PA0', 'PA1', 'PA2', 'PA3', 'PA8'])
for source in json.loads((root / 'dependencies.json').read_text())['sources']:
    assert re.fullmatch('[a-f0-9]{40}', source['commit'])
for msg in (root.parents[1] / 'interfaces/minesweeper_interfaces/msg').glob('*.msg'):
    assert '[]' not in msg.read_text() and 'string' not in msg.read_text()
makefile = (root / 'Makefile').read_text()
assert 'libmicroros.a' not in makefile and 'micro_ros/uros_app.c' not in makefile
assert 'Serial/Src/*.c' in makefile
print('project checks: XML, motor-disabled commissioning profile, immutable vendor pins, bounded messages passed')
