"""Exercise real serial I/O through a pseudo terminal, without robot hardware."""
import os
import select
import struct
import time

import pytest
import rclpy
from minesweeper_interfaces.msg import MotorCommand
from minesweeper_pi import stm32_bridge, stm32_protocol as wire


@pytest.fixture
def link(monkeypatch):
    master, slave = os.openpty()
    os.set_blocking(master, False)
    original = stm32_bridge.param
    monkeypatch.setattr(stm32_bridge, 'param', lambda node, name, default:
                        os.ttyname(slave) if name == 'port' else original(node, name, default))
    rclpy.init()
    node = stm32_bridge.Stm32Bridge()
    node.poll()
    yield node, master
    node.destroy_node()
    rclpy.shutdown()
    os.close(master)
    os.close(slave)


def wait_serial(node):
    assert select.select([node.connection.fileno()], [], [], .5)[0]
    node.poll()


def read_frames(master):
    until = time.monotonic()+.5
    while time.monotonic() < until:
        try:
            return wire.Parser().feed(os.read(master, 2048))
        except BlockingIOError:
            time.sleep(.001)
    return []


def status(node, flags=0):
    node.receive(wire.STATUS, wire.STATUS_DATA.pack(7, 0, 17, 1000, flags,
                                                   *([0.]*5), 600, 800, 8192, 8192))


def test_sync_and_command_serial_round_trip(link):
    node, master = link
    os.write(master, wire.encode(wire.SYNC_REQUEST, struct.pack('<I', 123)))
    wait_serial(node)
    frames = read_frames(master)
    assert len(frames) == 1 and frames[0][0] == wire.SYNC_RESPONSE
    token, epoch = struct.unpack('<IQ', frames[0][1])
    assert token == 123 and abs(epoch-time.time_ns()//1_000_000) < 500
    status(node)
    msg = MotorCommand(session=7, sequence=1, issued_at_ms=time.time_ns()//1_000_000,
                       arm=False, target=[0.]*5)
    node.on_command(msg)
    frames = read_frames(master)
    assert frames == [(wire.COMMAND, wire.command_payload(msg, msg.issued_at_ms))]


def test_disconnect_drops_pending_commands_and_requires_new_status(link):
    node, master = link
    status(node)
    node.disconnect('test')
    msg = MotorCommand(session=7, sequence=1, issued_at_ms=time.time_ns()//1_000_000,
                       arm=False, target=[0.]*5)
    node.on_command(msg)
    node.retry_at = 0
    node.poll()
    node.on_command(msg)
    assert read_frames(master) == []
    assert node.last_status is None


def test_bad_crc_is_not_published_as_status(link):
    node, master = link
    data = wire.STATUS_DATA.pack(7, 0, 17, 1000, 0, *([0.]*5), 600, 800, 8192, 8192)
    packet = bytearray(wire.encode(wire.STATUS, data))
    packet[-1] ^= 1
    os.write(master, packet)
    wait_serial(node)
    assert node.last_status is None
    os.write(master, wire.encode(wire.STATUS, data))
    wait_serial(node)
    assert node.last_status.session == 7 and not node.last_status.armed


def test_sensor_values_timestamps_and_unavailable_sensor_gating(link):
    node, _ = link
    encoders, imus, metal = [], [], []
    node.encoder_pub.publish = encoders.append
    node.imu_pub.publish = imus.append
    node.metal_pub.publish = metal.append
    epoch = 1792170000123
    encoder_data = wire.ENCODER_DATA.pack(epoch, 500, 2**40, -2**40, 1.5, -2.5)
    imu_data = wire.IMU_DATA.pack(epoch, 500, 1., 2., 3., 4., 5., 6.)
    node.receive(wire.ENCODERS, encoder_data)
    assert encoders == []  # No fresh status yet.
    status(node, flags=8|16|32)
    node.receive(wire.ENCODERS, encoder_data)
    node.receive(wire.IMU, imu_data)
    node.receive(wire.METAL, struct.pack('<f', 1.))
    assert list(encoders[0].ticks) == [2**40, -2**40]
    assert encoders[0].stamp.sec == epoch//1000 and encoders[0].stamp.nanosec == 123000000
    assert imus[0].linear_acceleration.z == 3. and imus[0].angular_velocity.y == 5.
    assert imus[0].orientation_covariance[0] == -1. and metal[0].data == 1.
    node.receive(wire.IMU, wire.IMU_DATA.pack(2**64-1, 500, *([0.]*6)))
    assert len(imus) == 1  # Invalid epoch cannot overflow a ROS timestamp.
    status(node, flags=0)
    node.receive(wire.ENCODERS, encoder_data)
    assert len(encoders) == 1
