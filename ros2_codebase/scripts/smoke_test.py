#!/usr/bin/env python3
"""Launch installed executables, drive mock hardware via Joy, assert DDS outputs."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage, Imu, Joy
from std_msgs.msg import Bool, Float64MultiArray, Int32
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


ROOT = Path(__file__).resolve().parents[1]


def main():
    artifacts = ROOT / 'artifacts'
    artifacts.mkdir(exist_ok=True)
    # Isolate from other robots and ROS CLI daemons on the development machine.
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('MINESWEEPER_TEST_DOMAIN', '87')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    config = artifacts / 'laptop_test.yaml'
    source_config = ROOT / 'laptop_ws/src/minesweeper_laptop/config/laptop.yaml'
    output_map = artifacts / 'mine_map.json'
    config.write_text(source_config.read_text().replace('~/.ros/minesweepers/mine_map.json', str(output_map)))
    processes, handles, checks = [], [], []
    report = {'checks': checks, 'ros_distro': os.environ.get('ROS_DISTRO'), 'success': False}
    node = None

    def launch(name, arguments):
        handle = (artifacts / (name+'.log')).open('w')
        handles.append(handle)
        process = subprocess.Popen(['ros2', *arguments], stdout=handle, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append((name, process))

    def check(name, condition):
        if not condition:
            raise AssertionError(name)
        checks.append(name)
        print('PASS:', name, flush=True)

    try:
        rclpy.init()
        node = Node('minesweeper_smoke_test')
        latest, count = {}, {}

        def capture(name):
            def callback(msg):
                latest[name] = msg
                count[name] = count.get(name, 0)+1
            return callback

        subscriptions = []
        for name, topic, kind, qos in [
            ('left', 'left_ticks', Int32, 10), ('right', 'right_ticks', Int32, 10),
            ('odom', 'odom', Odometry, 10), ('filtered', 'odometry/filtered', Odometry, 10),
            ('pose', 'pose_combined', PoseStamped, 10), ('imu', 'imu/data', Imu, qos_profile_sensor_data),
            ('image', 'camera/image/compressed', CompressedImage, qos_profile_sensor_data),
            ('camera', 'camera_detection', Bool, 10), ('duties', 'motor_duty_cycle', Float64MultiArray, 10),
            ('camera_ready', 'camera/ready', Bool, 10),
            ('aligned', 'camera/aligned', Bool, 10),
            ('gpio', 'gpio_motor_duty_cycle', Float64MultiArray, 10), ('cmd', 'cmd_vel', Twist, 10),
            ('map', 'mine_map', OccupancyGrid, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]:
            subscriptions.append(node.create_subscription(kind, topic, capture(name), qos))
        transforms = set()
        subscriptions.append(node.create_subscription(TFMessage, 'tf',
                             lambda msg: transforms.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms), 10))
        subscriptions.append(node.create_subscription(TFMessage, 'tf_static',
                             lambda msg: transforms.update((t.header.frame_id, t.child_frame_id) for t in msg.transforms),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        joy = node.create_publisher(Joy, 'joy', 10)
        metal = node.create_publisher(Bool, 'mock/metal_detected', 10)
        visible = node.create_publisher(Bool, 'mock/camera_mine_visible', 10)

        def spin(duration=0.1, driving=False, detecting=False, camera=True):
            until = time.monotonic()+duration
            while time.monotonic() < until:
                for name, process in processes:
                    if process.poll() is not None:
                        raise RuntimeError(f'{name} exited with {process.returncode}; see artifacts/{name}.log')
                if driving:
                    # 0.5 m/s, 0.1 rad/s, with deadman button 6 held.
                    joy.publish(Joy(axes=[0.1, 1.0], buttons=[0, 0, 0, 0, 0, 0, 1]))
                metal.publish(Bool(data=detecting))
                visible.publish(Bool(data=camera))
                rclpy.spin_once(node, timeout_sec=0.02)

        def until(predicate, timeout=15., **kwargs):
            deadline = time.monotonic()+timeout
            while not predicate():
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'ROS condition timed out; received: {list(latest)}')
                spin(**kwargs)

        launch('pi', ['launch', 'minesweeper_pi', 'pi.launch.py', 'mock:=true', 'camera:=false'])
        launch('camera', ['run', 'minesweeper_pi', 'camera', '--ros-args', '-p', 'mock:=true'])
        launch('laptop', ['launch', 'minesweeper_laptop', 'laptop.launch.py', 'mock:=true',
                          'teleop:=true', 'config:='+str(config)])
        launch('gpio', ['run', 'minesweeper_pi', 'motor_controller', '--ros-args', '-p', 'mock:=true'])
        expected = {'left', 'right', 'odom', 'filtered', 'pose', 'imu', 'image', 'camera',
                    'camera_ready', 'duties', 'gpio', 'cmd', 'map'}
        until(lambda: expected.issubset(latest))
        check('Both workspaces exchange all expected ROS 2 topics', expected.issubset(latest))
        grid = latest['map']
        check('Competition map covers 20 m by 20 m in 400 one-metre cells',
              (grid.info.width, grid.info.height, grid.info.resolution) == (20, 20, 1.0)
              and len(grid.data) == 400 and grid.header.frame_id == 'map')
        check('Mock camera sends JPEG image data', latest['image'].format == 'jpeg' and len(latest['image'].data) > 100)
        until(lambda: latest['camera'].data)
        check('Vision detects the synthetic mine from received JPEGs', latest['camera'].data)
        until(lambda: 'aligned' in latest and latest['aligned'].data)
        check('Vision detects crossing the gripper alignment line', latest['aligned'].data)
        check('No hardware is commanded before deadman input', max(abs(v) for v in latest['duties'].data) == 0.)
        baseline = latest['left'].data
        spin(1.6, driving=True, detecting=True)
        check('Joy/deadman input produces motion and encoder feedback', latest['left'].data > baseline+100)
        until(lambda: 50 in latest['map'].data, driving=True, detecting=True)
        check('Metal plus camera detection records a surface mine', 50 in latest['map'].data)
        until(lambda: latest['filtered'].pose.pose.position.x > 0.5, driving=True)
        pose = latest['pose']
        pose_x, pose_y = pose.pose.position.x, pose.pose.position.y
        pose_yaw = 2*math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)
        check('EKF publishes finite moving pose in radians', all(math.isfinite(v) for v in (pose_x, pose_y, pose_yaw))
              and pose_x > 0.4 and 0 < pose_yaw < math.pi)
        # Move into a new cell before allowing a buried classification.
        until(lambda: latest['filtered'].pose.pose.position.x > 2.0, timeout=12.,
              driving=True, detecting=False, camera=False)
        until(lambda: not latest['camera'].data, driving=True, camera=False)
        until(lambda: 100 in latest['map'].data, driving=True, detecting=True, camera=False)
        check('Metal without camera detection records a buried mine', 100 in latest['map'].data)
        check('Both motor implementations keep PWM duties within 100 percent',
              all(abs(v) <= 100. for key in ('duties', 'gpio') for v in latest[key].data))
        # Stop publishing Joy; teleop must expire it, then both drivers must stop.
        spin(1.3, camera=False)
        check('Joystick timeout publishes a zero velocity', latest['cmd'].linear.x == 0. and latest['cmd'].angular.z == 0.)
        check('Arduino and GPIO mock outputs stop on input loss',
              all(abs(v) < 1e-9 for key in ('duties', 'gpio') for v in latest[key].data))
        stopped = latest['left'].data
        spin(0.3, camera=False)
        check('Stopped robot produces no extra encoder ticks', latest['left'].data == stopped)
        check('TF links map, odom, base_link and imu_link',
              {('map', 'odom'), ('odom', 'base_link'), ('base_link', 'imu_link')}.issubset(transforms))
        for service in ('imu/reset', 'map/save'):
            client = node.create_client(Trigger, service)
            if not client.wait_for_service(timeout_sec=5.):
                raise TimeoutError('Missing service '+service)
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
            check(service+' succeeds', future.done() and future.result().success)
        covariance_client = node.create_client(SetParametersAtomically, 'ekf/set_parameters_atomically')
        if not covariance_client.wait_for_service(timeout_sec=5.):
            raise TimeoutError('Missing EKF parameter service')
        for values, expected_success in [([0.02, 0.02, 0.04, 0.03, 0.03], True), ([0.0]*5, False)]:
            request = SetParametersAtomically.Request()
            request.parameters = [Parameter('odom_variances', value=values).to_parameter_msg()]
            future = covariance_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
            check('EKF '+('accepts live covariance tuning' if expected_success else 'rejects zero measurement variances'),
                  future.done() and future.result().result.successful == expected_success)
        saved = json.loads(output_map.read_text())
        check('Saved map contains both surface and buried mine tables', bool(saved['surface']) and bool(saved['buried']))
        check('Saved map preserves field geometry, starting pose and metric detection positions',
              saved['field_width_m'] == saved['field_height_m'] == 20.0
              and saved['start_pose'] == {'x': 0.0, 'y': 0.0, 'heading_rad': 0.0}
              and bool(saved['detections']))
        until(lambda: latest['camera'].data and latest['aligned'].data, camera=True)
        camera_process = next(p for name, p in processes if name == 'camera')
        camera_process.send_signal(signal.SIGINT)
        camera_process.wait(timeout=12.)
        processes[:] = [(name, p) for name, p in processes if name != 'camera']
        until(lambda: not latest['camera'].data and not latest['aligned'].data, camera=False)
        check('Camera disconnection clears vision detection and alignment', not latest['camera'].data and not latest['aligned'].data)
        until(lambda: not latest['camera_ready'].data, camera=False)
        before = list(latest['map'].data)
        spin(3.0, driving=True, detecting=True, camera=False)
        check('Driving with metal detection during camera loss cannot invent buried cells',
              list(latest['map'].data) == before)
        # Independently run the finite ros2 run executable after stopping teleop,
        # so there is only one /cmd_vel publisher during this last check.
        laptop = next(p for name, p in processes if name == 'laptop')
        laptop.send_signal(signal.SIGINT)
        laptop.wait(timeout=12.)
        processes[:] = [(name, p) for name, p in processes if name != 'laptop']
        completed = subprocess.run(['ros2', 'run', 'minesweeper_laptop', 'command_demo',
                                    '--ros-args', '-p', 'duration:=1.0'],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10.)
        (artifacts/'command_demo.log').write_text(completed.stdout)
        check('Finite command_demo executable exits successfully', completed.returncode == 0)
        # Drivers also need to stop without a live teleop process sending zeros.
        direct_command = node.create_publisher(Twist, 'cmd_vel', 10)
        spin(0.3)
        command = Twist()
        command.linear.x = 0.2
        for _ in range(5):
            direct_command.publish(command)
            spin(0.05)
        check('Direct command reaches both motor drivers',
              all(any(abs(v) > 1. for v in latest[key].data) for key in ('duties', 'gpio')))
        spin(1.0)
        check('Driver watchdogs stop after the command publisher exits',
              all(abs(v) < 1e-9 for key in ('duties', 'gpio') for v in latest[key].data))
        report['message_counts'] = count
        report['sample_pose'] = {'x': pose_x, 'y': pose_y, 'theta_radians': pose_yaw}
        report['success'] = True
    except Exception as exc:
        report['error'] = str(exc)
        traceback.print_exc()
    finally:
        for _, process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for _, process in reversed(processes):
            try:
                process.wait(timeout=12.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for handle in handles:
            handle.close()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (artifacts/'smoke_report.json').write_text(json.dumps(report, indent=2)+'\n')
    if not report['success']:
        raise SystemExit(1)
    print(f"Passed {len(checks)} live checks. Report: {artifacts/'smoke_report.json'}")


if __name__ == '__main__':
    main()
