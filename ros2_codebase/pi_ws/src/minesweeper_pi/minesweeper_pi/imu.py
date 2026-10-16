"""MPU6050/MPU9250 I2C driver and guarded Madgwick IMU orientation filter."""
import math
import struct
import time

import numpy as np
from geometry_msgs.msg import TransformStamped, Twist, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .drive import wheel_speeds
from .runtime import param, run


def madgwick(q, gyro, acceleration, dt, beta=0.031):
    w, x, y, z = q
    gx, gy, gz = gyro
    derivative = 0.5 * np.array([-x*gx-y*gy-z*gz, w*gx+y*gz-z*gy,
                                 w*gy-x*gz+z*gx, w*gz+x*gy-y*gx])
    norm = np.linalg.norm(acceleration)
    if norm > 1e-12:
        ax, ay, az = np.asarray(acceleration) / norm
        residual = np.array([2*(x*z-w*y)-ax, 2*(w*x+y*z)-ay,
                             1-2*(x*x+y*y)-az])
        jacobian = np.array([[-2*y, 2*z, -2*w, 2*x],
                             [2*x, 2*w, 2*z, 2*y], [0, -4*x, -4*y, 0]])
        step = jacobian.T @ residual
        step_norm = np.linalg.norm(step)
        if step_norm > 1e-12:
            derivative -= beta * step / step_norm
    result = q + derivative * dt
    norm = np.linalg.norm(result)
    return result / norm if norm > 1e-12 else np.array([1., 0., 0., 0.])


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu')
        self.mock = param(self, 'mock', True)
        self.frame = param(self, 'frame_id', 'imu_link')
        parent = param(self, 'parent_frame', 'base_link')
        rate = param(self, 'rate', 60.0)
        self.beta = param(self, 'filter_gain', 0.031)
        self.samples = param(self, 'calibration_samples', 200)
        self.track = param(self, 'track_width', 0.55)
        self.max_speed = param(self, 'max_speed', 1.0)
        if rate <= 0 or self.samples < 0 or self.beta < 0:
            raise ValueError('invalid IMU rate, calibration count, or filter gain')
        self.bus = None
        self.address = param(self, 'address', 0x68)
        if not self.mock:
            from smbus2 import SMBus
            self.bus = SMBus(param(self, 'i2c_bus', 1))
            identity = self.bus.read_byte_data(self.address, 0x75)
            if identity not in (0x68, 0x69, 0x70, 0x71, 0x73):
                raise RuntimeError(f'Unexpected IMU WHO_AM_I: {identity:#x}')
            for register, value in ((0x6B, 0x01), (0x1A, 0x03), (0x1B, 0), (0x1C, 0), (0x19, 9)):
                self.bus.write_byte_data(self.address, register, value)
        self.command_w = 0.0
        self.last_command = float('-inf')
        self.command_timeout = param(self, 'command_timeout', 0.5)
        self.reset_filter()
        self.pub = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self.rpy_pub = self.create_publisher(Vector3Stamped, 'imu/rpy', qos_profile_sensor_data)
        self.create_subscription(Twist, 'cmd_vel', self.command, 10)
        self.create_service(Trigger, 'imu/reset', self.reset)
        # Mounting TF is fixed: world orientation in Imu must not be applied twice.
        self.tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id, transform.child_frame_id = parent, self.frame
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)
        self.create_timer(1.0 / rate, self.update)

    def reset_filter(self):
        self.q = np.array([1., 0., 0., 0.])
        self.bias = np.zeros(3)
        self.acc_bias = np.zeros(3)
        self.count = 0
        self.last_update = time.monotonic()

    def reset(self, request, response):
        self.reset_filter()
        response.success = True
        response.message = 'Orientation reset; stationary calibration restarted'
        return response

    def command(self, msg):
        left, right = wheel_speeds(msg.linear.x, msg.angular.z, self.track, self.max_speed)
        self.command_w = (right - left) / self.track
        self.last_command = time.monotonic()

    def update(self):
        now = time.monotonic()
        dt, self.last_update = min(now - self.last_update, 0.1), now
        if self.mock:
            acceleration = np.array([0., 0., 9.80665])
            gyro = np.array([0., 0., self.command_w if now-self.last_command < self.command_timeout else 0.])
        else:
            raw = struct.unpack('>7h', bytes(self.bus.read_i2c_block_data(self.address, 0x3B, 14)))
            acceleration = np.array(raw[:3]) * (9.80665 / 16384)
            gyro = np.array(raw[4:]) * (math.pi / (180 * 131.0))
            if self.count < self.samples:
                self.bias += gyro / self.samples
                self.acc_bias += (acceleration - [0., 0., 9.80665]) / self.samples
                self.count += 1
                return
            gyro -= self.bias
            acceleration -= self.acc_bias
        self.q = madgwick(self.q, gyro, acceleration, dt, self.beta)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        w, x, y, z = (float(v) for v in self.q)
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = w, x, y, z
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, gyro)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, acceleration)
        msg.orientation_covariance = [0.02, 0., 0., 0., 0.02, 0., 0., 0., 0.1]
        msg.angular_velocity_covariance = [0.01, 0., 0., 0., 0.01, 0., 0., 0., 0.01]
        msg.linear_acceleration_covariance = [0.1, 0., 0., 0., 0.1, 0., 0., 0., 0.1]
        self.pub.publish(msg)
        rpy = Vector3Stamped(header=msg.header)
        rpy.vector.x = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        rpy.vector.y = math.asin(max(-1., min(1., 2*(w*y-z*x))))
        rpy.vector.z = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        self.rpy_pub.publish(rpy)

    def destroy_node(self):
        if self.bus:
            self.bus.close()
        return super().destroy_node()


def main(args=None):
    run(ImuNode, args)
