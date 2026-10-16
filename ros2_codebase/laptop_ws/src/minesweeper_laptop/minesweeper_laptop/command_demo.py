"""Finite command publisher for mock-mode checks. Sends a stop before exiting."""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .runtime import param


def main(args=None):
    rclpy.init(args=args)
    node = Node('command_demo')
    duration = param(node, 'duration', 3.0)
    linear = param(node, 'linear', 0.2)
    angular = param(node, 'angular', 0.1)
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-start < duration:
            msg = Twist()
            msg.linear.x, msg.angular.z = linear, angular
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
