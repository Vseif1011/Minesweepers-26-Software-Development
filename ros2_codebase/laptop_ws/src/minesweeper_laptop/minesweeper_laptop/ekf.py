"""Five-state [x, y, yaw, v, omega] EKF; each measurement is consumed once."""
import math
import time

import numpy as np
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from .odometry import normalize, quaternion, yaw
from .runtime import param, run


class ExtendedKalmanFilter:
    def __init__(self):
        self.x = np.zeros(5)
        self.p = np.eye(5)

    def predict(self, dt):
        if dt <= 0:
            return
        _, _, theta, v, omega = self.x
        angle = theta + omega*dt/2
        c, s = math.cos(angle), math.sin(angle)
        f = np.eye(5)
        f[0, 2], f[0, 3], f[0, 4] = -dt*v*s, dt*c, -dt*dt*v*s/2
        f[1, 2], f[1, 3], f[1, 4] = dt*v*c, dt*s, dt*dt*v*c/2
        f[2, 4] = dt
        self.x[0] += dt*v*c
        self.x[1] += dt*v*s
        self.x[2] = normalize(theta + omega*dt)
        self.p = f @ self.p @ f.T + np.diag([0.002, 0.002, 0.005, 0.1, 0.1])*dt

    def correct(self, values, indices, variances):
        values, variances = np.asarray(values), np.asarray(variances)
        if not np.all(np.isfinite(values)) or not np.all(np.isfinite(variances)) or np.any(variances <= 0):
            return False
        h = np.eye(5)[indices]
        residual = values - h @ self.x
        for row, state in enumerate(indices):
            if state == 2:
                residual[row] = normalize(residual[row])
        r = np.diag(variances)
        covariance = h @ self.p @ h.T + r
        k = np.linalg.solve(covariance, h @ self.p).T
        self.x += k @ residual
        self.x[2] = normalize(self.x[2])
        # Joseph form keeps the covariance positive and symmetric after many updates.
        a = np.eye(5)-k @ h
        self.p = a @ self.p @ a.T + k @ r @ k.T
        self.p = (self.p+self.p.T)/2
        return True


class EkfNode(Node):
    def __init__(self):
        super().__init__('ekf')
        self.filter = ExtendedKalmanFilter()
        self.odom_frame = param(self, 'odom_frame', 'odom')
        self.base_frame = param(self, 'base_frame', 'base_link')
        self.publish_tf = param(self, 'publish_tf', True)
        self.sensor_timeout = param(self, 'sensor_timeout', 0.5)
        self.odom_variances = param(self, 'odom_variances', [0.03, 0.03, 0.05, 0.04, 0.04])
        self.imu_variances = param(self, 'imu_variances', [0.1, 0.01])
        if len(self.odom_variances) != 5 or len(self.imu_variances) != 2 or min(*self.odom_variances, *self.imu_variances) <= 0:
            raise ValueError('expected five positive odometry and two positive IMU variances')
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.add_post_set_parameters_callback(self.apply_parameters)
        self.last = self.get_clock().now()
        self.last_odom = float('-inf')
        self.odom_pending = self.imu_pending = None
        self.odom_stamp = self.imu_stamp = None
        self.pub = self.create_publisher(Odometry, 'odometry/filtered', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_combined', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_timer(0.05, self.update)

    def validate_parameters(self, parameters):
        for parameter in parameters:
            expected = {'odom_variances': 5, 'imu_variances': 2}.get(parameter.name)
            if expected:
                values = parameter.value
                if parameter.type_ != Parameter.Type.DOUBLE_ARRAY or len(values) != expected or any(
                        not isinstance(v, (float, int)) or not math.isfinite(v) or v <= 0 for v in values):
                    return SetParametersResult(successful=False, reason=f'{parameter.name} requires {expected} positive finite variances')
        return SetParametersResult(successful=True)

    def apply_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name in ('odom_variances', 'imu_variances'):
                setattr(self, parameter.name, list(parameter.value))

    def on_odom(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self.odom_stamp is not None and stamp <= self.odom_stamp:
            return
        self.odom_stamp = stamp
        self.odom_pending = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                             yaw(msg.pose.pose.orientation), msg.twist.twist.linear.x,
                             msg.twist.twist.angular.z]
        self.last_odom = time.monotonic()

    def on_imu(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self.imu_stamp is not None and stamp <= self.imu_stamp:
            return
        self.imu_stamp = stamp
        indices, values, variances = [], [], []
        q = msg.orientation
        if msg.orientation_covariance[0] >= 0 and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) < 0.01:
            indices.append(2)
            values.append(yaw(q))
            variances.append(self.imu_variances[0])
        if msg.angular_velocity_covariance[0] >= 0:
            indices.append(4)
            values.append(msg.angular_velocity.z)
            variances.append(self.imu_variances[1])
        self.imu_pending = values, indices, variances

    def update(self):
        now = self.get_clock().now()
        dt, self.last = (now-self.last).nanoseconds/1e9, now
        if dt <= 0:
            return
        if time.monotonic()-self.last_odom > self.sensor_timeout:
            self.filter.x[3:] = 0.0
        self.filter.predict(min(dt, 0.2))
        if self.odom_pending is not None:
            self.filter.correct(self.odom_pending, [0, 1, 2, 3, 4], self.odom_variances)
            self.odom_pending = None
        if self.imu_pending is not None:
            values, indices, variances = self.imu_pending
            if indices:
                self.filter.correct(values, indices, variances)
            self.imu_pending = None
        x, y, theta, v, omega = (float(value) for value in self.filter.x)
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id = now.to_msg(), self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation = quaternion(theta)
        self.pose_pub.publish(PoseStamped(header=msg.header, pose=msg.pose.pose))
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = v, omega
        for i, state_i in enumerate([0, 1, 2]):
            for j, state_j in enumerate([0, 1, 2]):
                ros_i, ros_j = [0, 1, 5][i], [0, 1, 5][j]
                msg.pose.covariance[ros_i*6+ros_j] = float(self.filter.p[state_i, state_j])
        for axis in (2, 3, 4):
            msg.pose.covariance[axis*7] = 1e6
        msg.twist.covariance[0], msg.twist.covariance[35] = float(self.filter.p[3, 3]), float(self.filter.p[4, 4])
        self.pub.publish(msg)
        if self.publish_tf:
            transform = TransformStamped(header=msg.header, child_frame_id=self.base_frame)
            transform.transform.translation.x, transform.transform.translation.y = x, y
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(transform)


def main(args=None):
    run(EkfNode, args)
