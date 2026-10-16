"""Encoder differential odometry; the EKF independently fuses the IMU."""
import math
import time

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int32
from minesweeper_interfaces.msg import EncoderTicks
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

from .runtime import param, run


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def quaternion(theta):
    return Quaternion(z=math.sin(theta/2), w=math.cos(theta/2))


def tick_delta(current, previous, bits=32):
    # Generated fixed arrays contain NumPy int64 scalars; use unbounded Python
    # integers so the modulo-2**64 calculation cannot overflow a C integer.
    return (int(current) - int(previous) + 2**(bits-1)) % 2**bits - 2**(bits-1)


class WheelOdometry:
    def __init__(self, ticks_left=3800., ticks_right=2255., track=0.55, counter_bits=32):
        if min(ticks_left, ticks_right, track) <= 0:
            raise ValueError('encoder scales and track must be positive')
        self.ticks_left, self.ticks_right, self.track = ticks_left, ticks_right, track
        self.counter_bits = counter_bits
        self.previous = None
        self.x = self.y = self.theta = 0.0

    def update(self, left, right, dt):
        if self.previous is None:
            self.previous = left, right
            return 0., 0., 0., 0.
        if dt <= 0:
            return 0., 0., 0., 0.
        dl = tick_delta(left, self.previous[0], self.counter_bits) / self.ticks_left
        dr = tick_delta(right, self.previous[1], self.counter_bits) / self.ticks_right
        self.previous = left, right
        distance, rotation = (dl+dr)/2, (dr-dl)/self.track
        # Exact constant-curvature integration, including a stable straight limit.
        scale = math.sin(rotation/2)/(rotation/2) if abs(rotation) > 1e-9 else 1.
        self.x += distance * scale * math.cos(self.theta+rotation/2)
        self.y += distance * scale * math.sin(self.theta+rotation/2)
        self.theta = normalize(self.theta+rotation)
        return distance/dt, rotation/dt, dl, dr


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')
        self.encoder_source = param(self, 'encoder_source', 'legacy')
        if self.encoder_source not in ('legacy', 'stm32'):
            raise ValueError('encoder_source must be legacy or stm32')
        self.model = WheelOdometry(param(self, 'ticks_per_meter_left', 3800.),
                                   param(self, 'ticks_per_meter_right', 2255.),
                                   param(self, 'track_width', 0.55),
                                   64 if self.encoder_source == 'stm32' else 32)
        self.sample_stamp = self.sample_uptime = None
        self.last_sample_received = float('-inf')
        self.base_frame = param(self, 'base_frame', 'base_link')
        self.odom_frame = param(self, 'odom_frame', 'odom')
        self.publish_tf = param(self, 'publish_tf', False)
        rate = param(self, 'rate', 20.)
        if rate <= 0:
            raise ValueError('rate must be positive')
        self.left = self.right = None
        self.left_new = self.right_new = False
        self.last = self.get_clock().now()
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'robot_2d', 10)
        self.dist_pub = self.create_publisher(Vector3Stamped, 'encoder_dist', 10)
        self.tf = TransformBroadcaster(self)
        if self.encoder_source == 'stm32':
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(EncoderTicks, '/encoders/ticks', self.on_encoders, qos)
        else:
            self.create_subscription(Int32, 'left_ticks', self.on_left, 10)
            self.create_subscription(Int32, 'right_ticks', self.on_right, 10)
            self.create_timer(1./rate, self.update)

    def on_encoders(self, msg):
        stamp = msg.stamp.sec*10**9 + msg.stamp.nanosec
        now = self.get_clock().now().nanoseconds
        if stamp <= 0 or not -0.1 <= (now-stamp)/1e9 <= 0.5:
            return
        # A reboot, time discontinuity or interrupted stream establishes a new
        # baseline; never interpret a counter reset as physical wheel movement.
        reboot = self.sample_uptime is not None and (
            (msg.sample_time_ms-self.sample_uptime) % 2**32 >= 2**31)
        gap = time.monotonic()-self.last_sample_received > 0.5
        if reboot or gap:
            self.model.previous = None
            self.sample_stamp = None
        if self.sample_stamp is not None and stamp <= self.sample_stamp:
            return
        dt = (stamp-self.sample_stamp)/1e9 if self.sample_stamp is not None else 0.0
        self.sample_stamp, self.sample_uptime = stamp, msg.sample_time_ms
        self.last_sample_received = time.monotonic()
        v, w, dl, dr = self.model.update(*msg.ticks, dt)
        self.publish_odometry(msg.stamp, v, w, dl, dr)

    def on_left(self, msg):
        self.left, self.left_new = msg.data, True

    def on_right(self, msg):
        self.right, self.right_new = msg.data, True

    def update(self):
        # Never invent a movement before both encoders arrive or reuse a partial pair.
        if not (self.left_new and self.right_new):
            return
        self.left_new = self.right_new = False
        now = self.get_clock().now()
        dt, self.last = (now-self.last).nanoseconds/1e9, now
        v, w, dl, dr = self.model.update(self.left, self.right, dt)
        self.publish_odometry(now.to_msg(), v, w, dl, dr)

    def publish_odometry(self, stamp, v, w, dl, dr):
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id = stamp, self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.model.x, self.model.y
        msg.pose.pose.orientation = quaternion(self.model.theta)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = v, w
        for i, variance in enumerate([0.03, 0.03, 1e6, 1e6, 1e6, 0.05]):
            msg.pose.covariance[i*7] = variance
        msg.twist.covariance[0], msg.twist.covariance[35] = 0.04, 0.04
        self.pub.publish(msg)
        self.pose_pub.publish(PoseStamped(header=msg.header, pose=msg.pose.pose))
        distances = Vector3Stamped(header=msg.header)
        distances.vector.x, distances.vector.y, distances.vector.z = dl, dr, (dl+dr)/2
        self.dist_pub.publish(distances)
        if self.publish_tf:
            transform = TransformStamped(header=msg.header, child_frame_id=self.base_frame)
            transform.transform.translation.x, transform.transform.translation.y = self.model.x, self.model.y
            transform.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(transform)


def main(args=None):
    run(OdometryNode, args)
