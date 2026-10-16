"""Adapt valid STM32 detector samples to the mapper's Boolean interface."""
import math
import time

from minesweeper_interfaces.msg import MotorStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32

from .runtime import param, run


class Stm32Detector(Node):
    def __init__(self):
        super().__init__('stm32_detector')
        self.threshold = param(self, 'threshold', 0.5)
        self.timeout = param(self, 'status_timeout', 0.5)
        if not math.isfinite(self.threshold) or not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('threshold must be finite and status_timeout positive and finite')
        self.valid = False
        self.status_time = float('-inf')
        self.pub = self.create_publisher(Bool, 'detection', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(MotorStatus, '/motor/status', self.on_status, qos)
        self.create_subscription(Float32, '/metal_detector/signal', self.on_signal, qos)

    def on_status(self, msg):
        self.valid = msg.metal_valid
        self.status_time = time.monotonic()

    def on_signal(self, msg):
        if self.valid and time.monotonic()-self.status_time < self.timeout and math.isfinite(msg.data):
            self.pub.publish(Bool(data=msg.data >= self.threshold))
        # Silence is unknown, not a negative detection. The mapper expires it.


def main(args=None):
    run(Stm32Detector, args)
