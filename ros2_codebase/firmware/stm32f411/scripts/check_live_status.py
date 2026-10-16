#!/usr/bin/env python3
"""Observe the motor-disabled commissioning firmware; never publish commands."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from minesweeper_interfaces.msg import MotorStatus
parser = argparse.ArgumentParser()
parser.add_argument('--seconds', type=float, default=8)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('stm32_commissioning_observer')
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)
samples = []


def receive(msg):
    samples.append({name: getattr(msg, name) for name in [
        'session', 'sequence', 'faults', 'uptime_ms', 'armed', 'ready',
        'hardware_ok', 'encoders_valid', 'imu_valid', 'metal_valid',
        'control_stack_free_words', 'uros_stack_free_words',
        'heap_free_bytes', 'heap_min_free_bytes']}
    )
    samples[-1]['duty'] = [float(value) for value in msg.duty]


node.create_subscription(MotorStatus, '/motor/status', receive, qos)
started = time.monotonic()
while time.monotonic()-started < args.seconds:
    rclpy.spin_once(node, timeout_sec=.1)
errors = []
if len(samples) < 20:
    errors.append(f'Only {len(samples)} status messages received')
if any(s['armed'] or s['hardware_ok'] or any(s['duty']) for s in samples):
    errors.append('Commissioning firmware reported enabled/unapproved motor state')
if samples and not all(s['faults'] & 1 for s in samples):
    errors.append('Hardware-unapproved fault was absent')
if len(samples) > 1 and samples[-1]['uptime_ms'] <= samples[0]['uptime_ms']:
    errors.append('Firmware uptime did not progress')
report = {'passed': not errors, 'errors': errors, 'count': len(samples),
          'seconds': args.seconds, 'first': samples[0] if samples else None,
          'last': samples[-1] if samples else None,
          'command_subscribers': node.count_subscribers('/motor/setpoints'),
          'topic_types': node.get_topic_names_and_types()}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({k: v for k, v in report.items() if k != 'topic_types'}, indent=2))
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if report['passed'] else 1)
