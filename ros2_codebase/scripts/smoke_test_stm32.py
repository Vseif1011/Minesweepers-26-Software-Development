#!/usr/bin/env python3
"""Exercise the installed laptop launch with synthetic native STM32 topics."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

# This source emulates telemetry only; it never sends a MotorCommand.
os.environ['ROS_DOMAIN_ID'] = os.environ.get('MINESWEEPER_STM32_TEST_DOMAIN', '88')
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import rclpy
from minesweeper_interfaces.msg import EncoderTicks, MotorStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32


ROOT = Path(__file__).resolve().parents[1]


def main():
    artifacts = ROOT / 'artifacts'
    artifacts.mkdir(exist_ok=True)
    config = artifacts / 'stm32_laptop_test.yaml'
    source = ROOT / 'laptop_ws/src/minesweeper_laptop/config/laptop.yaml'
    config.write_text(source.read_text().replace('~/.ros/minesweepers/mine_map.json',
                                               str(artifacts / 'stm32_mine_map.json')))
    report = {'success': False, 'checks': [], 'source': 'synthetic STM32 telemetry; no board/Agent'}
    process = node = None
    rclpy.init()
    try:
        with (artifacts / 'stm32_laptop.log').open('w') as log:
            process = subprocess.Popen(['ros2', 'launch', 'minesweeper_laptop', 'laptop.launch.py',
                                        'hardware:=stm32', 'mock:=true', 'vision:=false', 'config:='+str(config)],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        node = Node('synthetic_stm32_telemetry')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        enc = node.create_publisher(EncoderTicks, '/encoders/ticks', qos)
        imu = node.create_publisher(Imu, '/imu/data_raw', qos)
        metal = node.create_publisher(Float32, '/metal_detector/signal', qos)
        status = node.create_publisher(MotorStatus, '/motor/status', qos)
        latest = {}
        def capture(name):
            def callback(msg):
                latest[name] = msg
            return callback
        for name, topic, kind in [('odom', '/odom', Odometry), ('ekf', '/odometry/filtered', Odometry),
                                  ('metal', '/detection', Bool)]:
            node.create_subscription(kind, topic, capture(name), 10)
        start = time.monotonic()
        def spin(duration, sensors=True):
            until = time.monotonic()+duration
            while time.monotonic() < until:
                if process.poll() is not None:
                    raise RuntimeError('Laptop launch exited; inspect artifacts/stm32_laptop.log')
                cycle_deadline = time.monotonic()+0.02
                elapsed = time.monotonic()-start
                stamp = node.get_clock().now().to_msg()
                status.publish(MotorStatus(session=7, hardware_ok=False, armed=False, metal_valid=sensors))
                if sensors:
                    enc.publish(EncoderTicks(stamp=stamp, sample_time_ms=int(elapsed*1000),
                                             ticks=[2**40+int(elapsed*760), 2**40+int(elapsed*451)]))
                    sample = Imu()
                    sample.header.stamp, sample.header.frame_id = stamp, 'imu_link'
                    sample.orientation_covariance[0] = -1.
                    sample.angular_velocity.z = 0.2
                    imu.publish(sample)
                metal.publish(Float32(data=1.0))
                while time.monotonic() < cycle_deadline:
                    rclpy.spin_once(node, timeout_sec=max(0., cycle_deadline-time.monotonic()))
        def check(name, condition):
            if not condition:
                raise AssertionError(name)
            report['checks'].append(name)
            print('PASS:', name, flush=True)
        spin(3.0)
        check('Native paired 64-bit encoder data reaches odometry',
              'odom' in latest and latest['odom'].pose.pose.position.x > 0.1)
        check('Raw STM32 gyro reaches the EKF without an invented orientation',
              'ekf' in latest and latest['ekf'].twist.twist.angular.z > 0.01)
        check('Valid native detector signal reaches mapping interface',
              'metal' in latest and latest['metal'].data)
        check('STM32 launch creates no legacy tick or filtered-IMU publishers',
              node.count_publishers('/left_ticks') == 0 and node.count_publishers('/imu/data') == 0)
        check('Telemetry launch sends no STM32 motor commands', node.count_publishers('/motor/setpoints') == 0)
        stamp = latest['odom'].header.stamp
        detections = []
        node.create_subscription(Bool, '/detection', lambda msg: detections.append(msg.data), 10)
        spin(0.3, sensors=False)  # drain already-transmitted valid samples
        detections.clear()
        spin(0.8, sensors=False)
        check('Unavailable sensors produce no invented zero-motion or negative-detector samples',
              latest['odom'].header.stamp == stamp and detections == [])
        report['success'] = True
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=12.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (artifacts / 'stm32_integration_report.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
