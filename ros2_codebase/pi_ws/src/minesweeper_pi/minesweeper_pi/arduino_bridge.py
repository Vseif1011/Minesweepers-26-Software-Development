"""Legacy Arduino Mega ASCII bridge and mock drive; never use with STM32.

The STM32 speaks micro-ROS XRCE-DDS through micro_ros_agent, not this protocol.
Real STM32 deployment is selected by pi.launch.py with mock:=false.
"""
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, Int16, Int32

from .drive import encode_command, motor_duties, parse_telemetry, wheel_speeds
from .runtime import param, run


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.mock = param(self, 'mock', True)
        self.drive_motors = param(self, 'drive_motors', True)
        self.track = param(self, 'track_width', 0.55)
        self.max_speed = param(self, 'max_speed', 1.0)
        self.ticks_l = param(self, 'ticks_per_meter_left', 3800.0)
        self.ticks_r = param(self, 'ticks_per_meter_right', 2255.0)
        self.timeout = param(self, 'command_timeout', 0.5)
        rate = param(self, 'rate', 20.0)
        if min(self.track, self.max_speed, self.ticks_l, self.ticks_r, rate, self.timeout) <= 0:
            raise ValueError('drive calibration, rate, and timeout must be positive')
        self.serial = None
        if not self.mock:
            import serial
            self.serial = serial.Serial(param(self, 'port', '/dev/ttyACM0'),
                                        param(self, 'baud', 115200), timeout=0, write_timeout=0.1)
        self.buffer = bytearray()
        self.linear = self.angular = 0.0
        self.controller = 9
        self.controller_time = self.last_command = float('-inf')
        self.last_update = time.monotonic()
        self.left = self.right = 0.0
        self.mock_metal = False
        self.left_pub = self.create_publisher(Int32, 'left_ticks', 10)
        self.right_pub = self.create_publisher(Int32, 'right_ticks', 10)
        self.metal_pub = self.create_publisher(Bool, 'detection', 10)
        self.duty_pub = self.create_publisher(Float64MultiArray, 'motor_duty_cycle', 10)
        self.create_subscription(Twist, 'cmd_vel', self.command, 10)
        self.create_subscription(Int16, 'controller', self.control, 10)
        if self.mock:
            self.create_subscription(Bool, 'mock/metal_detected', self.set_metal, 10)
        self.create_timer(1.0 / rate, self.update)
        self.get_logger().info(f'Arduino bridge: {"mock" if self.mock else "serial"}')

    def command(self, msg):
        self.linear, self.angular = msg.linear.x, msg.angular.z
        self.last_command = time.monotonic()

    def control(self, msg):
        if msg.data in (0, 5, 6, 7, 8, 9, 10, 11):
            self.controller = msg.data
            self.controller_time = time.monotonic()

    def set_metal(self, msg):
        self.mock_metal = msg.data

    def publish_feedback(self, left, right, metal):
        # Int32 agrees with the new firmware and removes the legacy Int16 truncation.
        wrap = lambda n: (int(n) + 2**31) % 2**32 - 2**31
        self.left_pub.publish(Int32(data=wrap(left)))
        self.right_pub.publish(Int32(data=wrap(right)))
        self.metal_pub.publish(Bool(data=metal))

    def update(self):
        now = time.monotonic()
        dt, self.last_update = min(now - self.last_update, 0.2), now
        linear, angular = self.linear, self.angular
        if now - self.last_command > self.timeout:
            linear = angular = 0.0
        left, right = wheel_speeds(linear, angular, self.track, self.max_speed)
        # Reconstruct the saturated command so mock and physical drives agree.
        linear, angular = (left + right) / 2, (right - left) / self.track
        controller = self.controller
        if controller in (7, 8) and now - self.controller_time > self.timeout:
            controller = 9
        self.duty_pub.publish(Float64MultiArray(data=motor_duties(left, right, self.max_speed)))
        if self.mock:
            self.left += left * dt * self.ticks_l
            self.right += right * dt * self.ticks_r
            self.publish_feedback(self.left, self.right, self.mock_metal)
            return
        try:
            self.serial.write(encode_command(linear if self.drive_motors else 0.0,
                                             angular if self.drive_motors else 0.0, controller))
            self.buffer.extend(self.serial.read(min(self.serial.in_waiting, 4096)))
            if len(self.buffer) > 4096:
                self.buffer.clear()
                self.get_logger().warning('Discarded oversized serial buffer')
            while b'\n' in self.buffer:
                line, _, remainder = self.buffer.partition(b'\n')
                self.buffer = bytearray(remainder)
                try:
                    self.publish_feedback(*parse_telemetry(line.decode('ascii')))
                except (ValueError, UnicodeError):
                    self.get_logger().warning('Discarded malformed Arduino telemetry')
        except OSError as exc:
            self.get_logger().error(f'Serial I/O failed: {exc}')
            raise

    def destroy_node(self):
        if self.serial is not None:
            try:
                self.serial.write(encode_command(0.0, 0.0, 9))
            finally:
                self.serial.close()
        return super().destroy_node()


def main(args=None):
    run(ArduinoBridge, args)
