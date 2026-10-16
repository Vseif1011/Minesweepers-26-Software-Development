"""ROS 2 topics to direct STM32 USB serial. Never creates or replays commands."""
import math
import struct
import time

import serial
from rclpy.node import Node
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.qos import QoSProfile, ReliabilityPolicy
from minesweeper_interfaces.msg import EncoderTicks, MotorCommand, MotorStatus
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from . import stm32_protocol as wire
from .runtime import param, run


class Stm32Bridge(Node):
    def __init__(self):
        # Port/baud are startup settings; avoid six unused parameter services on Pi 3.
        super().__init__('stm32_bridge', start_parameter_services=False)
        self.port = param(self, 'port', '/dev/ttyACM0')
        self.baud = param(self, 'baud', 115200)
        self.connection = None
        self.parser = wire.Parser()
        self.retry_at = self.last_rx = self.status_at = 0.
        self.last_status = None
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        events = PublisherEventCallbacks(use_default_callbacks=False)
        self.status_pub = self.create_publisher(MotorStatus, '/motor/status', self.qos, event_callbacks=events)
        self.encoder_pub = self.create_publisher(EncoderTicks, '/encoders/ticks', self.qos, event_callbacks=events)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', self.qos, event_callbacks=events)
        self.metal_pub = self.create_publisher(Float32, '/metal_detector/signal', self.qos, event_callbacks=events)
        self.create_subscription(MotorCommand, '/motor/setpoints', self.on_command, self.qos,
                                 event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False))
        self.create_timer(.02, self.poll)

    def disconnect(self, reason):
        if self.connection is not None:
            self.connection.close()
        self.connection = None
        self.parser = wire.Parser()
        self.last_status = None
        self.retry_at = time.monotonic() + 1.
        self.get_logger().warning(f'STM32 serial disconnected: {reason}')

    def send(self, kind, payload):
        if self.connection is None:
            return False
        try:
            packet = wire.encode(kind, payload)
            if self.connection.write(packet) != len(packet):
                raise serial.SerialTimeoutException('Partial serial write')
            return True
        except (serial.SerialException, OSError) as exc:
            self.disconnect(str(exc))
            return False

    def on_command(self, msg):
        if self.connection is None:
            return
        if (self.last_status is None or time.monotonic() - self.status_at > .15 or
                msg.session != self.last_status.session):
            self.get_logger().warning('Dropped command without fresh matching STM32 status')
            return
        try:
            payload = wire.command_payload(msg, time.time_ns() // 1_000_000)
        except (ValueError, OverflowError, struct.error) as exc:
            self.get_logger().warning(str(exc))
            return
        self.send(wire.COMMAND, payload)

    @staticmethod
    def stamp(msg, epoch):
        msg.sec, millis = divmod(epoch, 1000)
        msg.nanosec = millis * 1_000_000

    def receive(self, kind, payload):
        if kind == wire.SYNC_REQUEST and len(payload) == 4:
            token, = struct.unpack('<I', payload)
            self.send(wire.SYNC_RESPONSE, struct.pack('<IQ', token, time.time_ns() // 1_000_000))
        elif kind == wire.STATUS and len(payload) == wire.STATUS_DATA.size:
            values = wire.STATUS_DATA.unpack(payload)
            msg = MotorStatus()
            msg.session, msg.sequence, msg.faults, msg.uptime_ms = values[:4]
            flags = values[4]
            for bit, name in enumerate(('armed', 'ready', 'hardware_ok', 'encoders_valid',
                                        'imu_valid', 'metal_valid')):
                setattr(msg, name, bool(flags & (1 << bit)))
            if not all(math.isfinite(value) for value in values[5:10]):
                return
            msg.duty = list(values[5:10])
            # Legacy schema name; now reports serial-task stack rather than micro-ROS.
            (msg.control_stack_free_words, msg.uros_stack_free_words,
             msg.heap_free_bytes, msg.heap_min_free_bytes) = values[10:]
            self.last_status, self.status_at = msg, time.monotonic()
            self.status_pub.publish(msg)
        elif kind == wire.ENCODERS and len(payload) == wire.ENCODER_DATA.size:
            if not self.sensor_valid('encoders_valid'):
                return
            epoch, sample, left, right, lv, rv = wire.ENCODER_DATA.unpack(payload)
            if not 0 < epoch // 1000 <= 2147483647 or not all(math.isfinite(value) for value in (lv, rv)):
                return
            msg = EncoderTicks(sample_time_ms=sample, ticks=[left, right], velocity_rad_s=[lv, rv])
            self.stamp(msg.stamp, epoch)
            self.encoder_pub.publish(msg)
        elif kind == wire.IMU and len(payload) == wire.IMU_DATA.size:
            if not self.sensor_valid('imu_valid'):
                return
            epoch, _, ax, ay, az, gx, gy, gz = wire.IMU_DATA.unpack(payload)
            if not 0 < epoch // 1000 <= 2147483647 or not all(math.isfinite(value) for value in (ax, ay, az, gx, gy, gz)):
                return
            msg = Imu()
            self.stamp(msg.header.stamp, epoch)
            msg.header.frame_id = 'imu_link'
            msg.orientation_covariance[0] = -1.
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ax, ay, az
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gx, gy, gz
            self.imu_pub.publish(msg)
        elif kind == wire.METAL and len(payload) == 4 and self.sensor_valid('metal_valid'):
            value, = struct.unpack('<f', payload)
            if math.isfinite(value):
                self.metal_pub.publish(Float32(data=value))

    def sensor_valid(self, field):
        return (self.last_status is not None and time.monotonic() - self.status_at < .15 and
                getattr(self.last_status, field))

    def poll(self):
        now = time.monotonic()
        if self.connection is None:
            if now < self.retry_at:
                return
            try:
                self.connection = serial.Serial(self.port, self.baud, timeout=0,
                                                write_timeout=.02, exclusive=True)
                self.connection.reset_input_buffer()
                self.last_rx = now
                self.get_logger().info(f'Direct STM32 serial connected: {self.port}')
            except (serial.SerialException, OSError) as exc:
                self.disconnect(str(exc))
            return
        try:
            if self.parser.buffer and now - self.last_rx > .1:
                self.parser = wire.Parser()
            data = self.connection.read(min(2048, self.connection.in_waiting))
            if data:
                self.last_rx = now
                for kind, payload in self.parser.feed(data):
                    self.receive(kind, payload)
            elif now - self.last_rx > 2.:
                self.disconnect('No STM32 traffic; check direct-serial firmware')
        except (serial.SerialException, OSError) as exc:
            self.disconnect(str(exc))

    def destroy_node(self):
        if self.connection is not None:
            self.connection.close()
        return super().destroy_node()


def main(args=None):
    run(Stm32Bridge, args)
