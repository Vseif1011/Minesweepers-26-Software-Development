"""Exercise the real pyserial path against a pseudo terminal, without an Arduino."""
import os
import pty
import select
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist

from minesweeper_pi.arduino_bridge import ArduinoBridge


def test_real_serial_backend_handles_fragmented_frames_and_watchdog():
    pytest.importorskip('serial')
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    rclpy.init(args=['--ros-args', '-p', 'mock:=false', '-p', 'port:='+os.ttyname(slave)])
    node = None
    def read_until(expected):
        wire = b''
        deadline = time.monotonic()+1.
        while expected not in wire and time.monotonic() < deadline:
            if select.select([master], [], [], 0.02)[0]:
                wire += os.read(master, 4096)
        return wire

    try:
        node = ArduinoBridge()
        received = []
        node.publish_feedback = lambda *values: received.append(values)
        os.write(master, b'T,123,')
        node.update()
        assert received == []
        os.write(master, b'456,1\nT,bad,3,0\nT,-3,7,0\n')
        # pyserial's input buffer may update on the next scheduler cycle.
        for _ in range(20):
            node.update()
            if len(received) == 2:
                break
            select.select([node.serial.fileno()], [], [], 0.01)
        assert received == [(123, 456, True), (-3, 7, False)]
        command = Twist()
        command.linear.x = 0.25
        node.command(command)
        node.update()
        expected = b'C,0.25000,0.00000,9\n'
        assert expected in read_until(expected)
        node.last_command = float('-inf')
        node.update()
        expected = b'C,0.00000,0.00000,9\n'
        assert expected in read_until(expected)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
