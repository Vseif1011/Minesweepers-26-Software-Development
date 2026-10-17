#!/usr/bin/env python3
"""Installed STM32 teleop with synthetic Joy/MCU acknowledgements, never hardware."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('MINESWEEPER_JOYSTICK_TEST_DOMAIN', '96')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy
from minesweeper_interfaces.msg import MotorCommand, MotorStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Joy


ROOT = Path(__file__).resolve().parents[1]


def main():
    output = ROOT/'artifacts/joystick_motor_setup'
    output.mkdir(parents=True, exist_ok=True)
    log = (output/'ros_joystick_smoke.log').open('w')
    process = None
    rclpy.init()
    node = Node('synthetic_joystick_and_mcu')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    status = MotorStatus(session=77, hardware_ok=True)
    status_pub = node.create_publisher(MotorStatus, '/motor/status', qos)
    joy_pub = node.create_publisher(Joy, '/test_joy', qos_profile_sensor_data)
    commands, checks = [], []
    report = {'success': False, 'checks': checks, 'source': 'Synthetic MCU; isolated ROS domain; no serial device'}

    def receive(msg):
        if msg.session != status.session:
            return  # In-flight packets from the retired MCU session are not accepted.
        assert msg.sequence > status.sequence
        assert -20 <= time.time_ns()//1_000_000-msg.issued_at_ms <= 100
        if not msg.arm:
            assert not any(msg.target)
            status.armed, status.ready = False, True
        elif not status.armed:
            assert status.ready and not any(msg.target), 'Nonzero or unprepared arm request'
            status.armed, status.ready = True, False
        status.sequence = msg.sequence
        commands.append(msg)

    node.create_subscription(MotorCommand, '/motor/setpoints', receive, qos)

    def spin(duration, axes=None, held=False, publish_joy=True):
        until = time.monotonic()+duration
        while time.monotonic() < until:
            assert process.poll() is None, 'Teleop exited unexpectedly'
            deadline = time.monotonic()+0.04
            status.uptime_ms += 40
            status_pub.publish(status)
            if publish_joy:
                msg = Joy(axes=axes or [0.]*6, buttons=[0, int(held)]+[0]*10)
                msg.header.stamp = node.get_clock().now().to_msg()
                joy_pub.publish(msg)
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=max(0., deadline-time.monotonic()))

    def until(predicate, **kwargs):
        deadline = time.monotonic()+8
        while not predicate():
            if time.monotonic() > deadline:
                raise AssertionError('Timed out waiting for joystick/MCU state')
            spin(.08, **kwargs)

    def check(name, condition):
        assert condition, name
        checks.append(name)
        print('PASS:', name, flush=True)

    try:
        process = subprocess.Popen(['ros2', 'run', 'minesweeper_laptop', 'stm32_teleop', '--ros-args',
                                    '-p', 'dry_run:=false', '-p', 'joystick_device:=""',
                                    '-r', 'joy:=/test_joy'], stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        until(lambda: bool(commands))
        check('Startup only sends zero/disarm commands', not any(m.arm or any(m.target) for m in commands))
        until(lambda: status.armed, held=True)
        check('MCU acknowledges zero/disarm then zero/arm before driving', status.armed)
        for name, axes, expected in [
            ('Forward', [0., 1., 0., 0., 0., 0.], [.2, .2, .2, .2, 0.]),
            ('Reverse', [0., -1., 0., 0., 0., 0.], [-.2, -.2, -.2, -.2, 0.]),
            ('Twist right', [0., 0., -1., 0., 0., 0.], [.2, .2, -.2, -.2, 0.]),
            ('Twist left', [0., 0., 1., 0., 0., 0.], [-.2, -.2, .2, .2, 0.]),
            ('Base lever forward', [0., 0., 0., 1., 0., 0.], [0., 0., 0., 0., .15]),
            ('Base lever reverse', [0., 0., 0., -1., 0., 0.], [0., 0., 0., 0., -.15]),
        ]:
            spin(.25, axes=axes, held=True)
            check(name+' produces the five expected signed duties',
                  commands[-1].arm and all(abs(a-b)<1e-5 for a,b in zip(commands[-1].target, expected)))
        spin(.2, axes=[0., 1., 0., 1., 0., 0.], held=False)
        check('Releasing button 2 stops all five outputs', not status.armed and not any(commands[-1].target))
        spin(.15)
        until(lambda: status.armed, held=True)
        spin(.2, axes=[0., 1., 0., 0., 0., 0.], held=True)
        spin(.4, publish_joy=False)
        check('Lost joystick messages disarm despite live MCU status', not status.armed and not any(commands[-1].target))
        spin(.2, held=True)
        check('Held button after interruption cannot rearm', not status.armed)
        spin(.12)
        until(lambda: status.armed, held=True)
        status.session += 1
        status.sequence = 0
        status.armed = status.ready = False
        spin(.25, held=True)
        check('New MCU session cannot restart a held joystick', not status.armed)
        report['success'], report['command_count'] = True, len(commands)
    finally:
        if process:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        (output/'ros_joystick_report.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
