"""STM32 message handling, including real DDS sensor QoS and missing sensors."""
import time

import pytest
import rclpy
from builtin_interfaces.msg import Time
from minesweeper_interfaces.msg import EncoderTicks, MotorStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32

from minesweeper_laptop.odometry import OdometryNode
from minesweeper_laptop.stm32_detector import Stm32Detector


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_stm32_encoder_pair_preserves_64_bits_stamps_and_reboots(ros_context):
    # Use normal ROS parameter overrides, without changing global CLI arguments.
    rclpy.shutdown()
    rclpy.init(args=['--ros-args', '-p', 'encoder_source:=stm32'])
    node = OdometryNode()
    outputs = []
    node.publish_odometry = lambda *values: outputs.append(values)
    try:
        now = node.get_clock().now().nanoseconds
        def sample(offset_ms, uptime, left, right):
            stamp = now + offset_ms*10**6
            return EncoderTicks(stamp=Time(sec=stamp//10**9, nanosec=stamp % 10**9),
                                sample_time_ms=uptime, ticks=[left, right])
        baseline = 2**40
        node.on_encoders(sample(-300, 1000, baseline, baseline))
        node.on_encoders(sample(-200, 1100, baseline+380, baseline+226))
        assert node.model.x == pytest.approx((380/3800+226/2255)/2, abs=1e-6)
        assert outputs[-1][0] == sample(-200, 1100, 0, 0).stamp
        previous_x = node.model.x
        node.on_encoders(sample(-100, 10, 0, 0))  # MCU restarted
        assert node.model.x == previous_x
        assert outputs[-1][1:] == (0., 0., 0., 0.)
        node.on_encoders(sample(-1000, 20, 38000, 22550))  # stale data
        assert node.model.x == previous_x
        node.last_sample_received = float('-inf')  # interrupted stream
        node.on_encoders(sample(0, 100, 38000, 22550))
        assert node.model.x == previous_x
    finally:
        node.destroy_node()


def test_stm32_detector_requires_fresh_valid_status_and_finite_signal(ros_context):
    node = Stm32Detector()
    outputs = []
    class Capture:
        def publish(self, msg):
            outputs.append(msg.data)
    node.pub = Capture()
    try:
        node.on_signal(Float32(data=1.0))
        assert outputs == []  # no valid sensor status
        node.on_status(MotorStatus(metal_valid=True))
        node.on_signal(Float32(data=1.0))
        node.on_signal(Float32(data=0.0))
        assert outputs == [True, False]
        node.on_signal(Float32(data=float('nan')))
        node.status_time = float('-inf')
        node.on_signal(Float32(data=1.0))
        node.on_status(MotorStatus(metal_valid=False))
        node.on_signal(Float32(data=1.0))
        assert outputs == [True, False]
    finally:
        node.destroy_node()


def test_stm32_best_effort_dds_connects_to_native_odometry(ros_context):
    rclpy.shutdown()
    rclpy.init(args=['--ros-args', '-p', 'encoder_source:=stm32'])
    node = OdometryNode()
    source = Node('synthetic_stm32')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher = source.create_publisher(EncoderTicks, '/encoders/ticks', qos)
    try:
        deadline = time.monotonic()+5.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert publisher.get_subscription_count() == 1
        for _ in range(10):
            publisher.publish(EncoderTicks(stamp=source.get_clock().now().to_msg(),
                                          sample_time_ms=100, ticks=[2**40, -2**40]))
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.model.previous is not None:
                break
        assert node.model.previous == (2**40, -2**40)
    finally:
        source.destroy_node()
        node.destroy_node()
