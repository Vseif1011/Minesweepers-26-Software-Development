"""ROS Joy teleoperation with configurable axes, deadman, and input timeout."""
import math
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

from .runtime import param, run


def axis_value(axes, index, deadzone):
    if index < 0 or index >= len(axes) or not math.isfinite(axes[index]):
        return 0.0
    value = max(-1., min(1., axes[index]))
    return value if abs(value) > deadzone else 0.0


class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.linear_axis = param(self, 'linear_axis', 1)
        self.angular_axis = param(self, 'angular_axis', 0)
        self.deadman = param(self, 'deadman_button', 6)
        self.linear_scale = param(self, 'linear_scale', 0.5)
        self.angular_scale = param(self, 'angular_scale', 1.0)
        self.deadzone = param(self, 'deadzone', 0.08)
        self.timeout = param(self, 'input_timeout', 0.5)
        self.mapping = {name: param(self, name+'_button', default) for name, default in
                        [('arm_up', 0), ('arm_down', 3), ('release', 1), ('hold', 2),
                         ('speed_up', 4), ('speed_down', 5)]}
        if self.deadman < 0 or self.timeout <= 0 or not 0 <= self.deadzone < 1:
            raise ValueError('deadman button, timeout, or deadzone invalid')
        self.speed = 1.0
        self.joy = Joy()
        self.previous = []
        self.last = float('-inf')
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.controller_pub = self.create_publisher(Int16, 'controller', 10)
        self.create_subscription(Joy, 'joy', self.on_joy, qos_profile_sensor_data)
        self.create_timer(0.05, self.update)

    @staticmethod
    def pressed(buttons, index):
        return 0 <= index < len(buttons) and buttons[index] != 0

    def on_joy(self, msg):
        enabled = self.pressed(msg.buttons, self.deadman)
        if enabled:
            for name, code in [('release', 10), ('hold', 11), ('speed_up', 5), ('speed_down', 6)]:
                button = self.mapping[name]
                if self.pressed(msg.buttons, button) and not self.pressed(self.previous, button):
                    if code == 5:
                        self.speed = min(1.0, self.speed+0.1)
                    elif code == 6:
                        self.speed = max(0.1, self.speed-0.1)
                    self.controller_pub.publish(Int16(data=code))
        self.previous = list(msg.buttons)
        self.joy, self.last = msg, time.monotonic()

    def update(self):
        msg, controller = Twist(), 9
        if time.monotonic()-self.last < self.timeout and self.pressed(self.joy.buttons, self.deadman):
            msg.linear.x = axis_value(self.joy.axes, self.linear_axis, self.deadzone)*self.linear_scale*self.speed
            msg.angular.z = axis_value(self.joy.axes, self.angular_axis, self.deadzone)*self.angular_scale*self.speed
            up = self.pressed(self.joy.buttons, self.mapping['arm_up'])
            down = self.pressed(self.joy.buttons, self.mapping['arm_down'])
            controller = 7 if up and not down else 8 if down and not up else 9
        self.velocity_pub.publish(msg)
        self.controller_pub.publish(Int16(data=controller))


def main(args=None):
    run(Teleop, args)
