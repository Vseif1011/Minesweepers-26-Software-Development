"""Five-channel PWM teleoperation with neutral/deadman and MCU acknowledgement."""
import math
from pathlib import Path
import time

from minesweeper_interfaces.msg import MotorCommand, MotorStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

from .runtime import param, run


def mix_targets(forward, turn, arm, left, right, signs, drive_limit, arm_limit):
    """Positive turn is CCW; preserve the drive ratio when mixing saturates."""
    if not all(math.isfinite(v) for v in (forward, turn, arm)):
        raise ValueError('nonfinite joystick demand')
    l, r = forward-turn, forward+turn
    divisor = max(1.0, abs(l), abs(r))
    targets = [0.0]*5
    for channels, value in ((left, l), (right, r)):
        for index in channels:
            targets[index] = signs[index]*drive_limit*value/divisor
    targets[4] = signs[4]*arm_limit*max(-1.0, min(1.0, arm))
    return targets


class MotorSession:
    """One fresh command stream; connection recovery never resumes a held button."""

    def __init__(self, input_timeout=0.25, status_timeout=0.15):
        self.input_timeout, self.status_timeout = input_timeout, status_timeout
        self.status = None
        self.status_at = self.joy_at = float('-inf')
        self.session, self.sequence = None, 0
        self.held = False
        self.targets = [0.0]*5
        self.invalidate()

    def invalidate(self):
        self.locked, self.phase = True, 'off'
        self.ack_sequence = None
        self.phase_started = None

    def on_status(self, msg, now):
        if not msg.session or not all(math.isfinite(v) for v in msg.duty):
            return
        if msg.session != self.session:
            self.session, self.sequence = msg.session, 0
            self.invalidate()
        elif self.status is not None and msg.sequence < self.status.sequence:
            return
        self.status, self.status_at = msg, now

    def on_joy(self, targets, held, neutral, now):
        self.targets, self.joy_at = list(targets), now
        if not held:
            self.phase, self.ack_sequence = 'off', None
            self.phase_started = None
            self.locked = not neutral
        elif not self.held:
            if self.locked or not neutral:
                self.invalidate()
            else:
                self.phase, self.ack_sequence = 'prepare', None
                self.phase_started = None
        self.held = held

    def packet(self, arm, targets, epoch_ms):
        self.sequence += 1
        return MotorCommand(session=self.session, sequence=self.sequence,
                            issued_at_ms=epoch_ms, arm=arm, target=targets)

    def step(self, now, epoch_ms):
        zero = [0.0]*5
        if self.status is None or now-self.status_at >= self.status_timeout:
            self.invalidate()
            return None
        if self.sequence >= 0xffffffff:
            self.invalidate()
            return None
        if now-self.joy_at >= self.input_timeout or not self.status.hardware_ok:
            self.invalidate()
        if self.locked or not self.held:
            return self.packet(False, zero, epoch_ms)
        if self.phase_started is None:
            self.phase_started = now
        if self.phase != 'active' and now-self.phase_started > 1.0:
            self.invalidate()
            return self.packet(False, zero, epoch_ms)
        acknowledged = self.ack_sequence is not None and self.status.sequence >= self.ack_sequence
        if self.phase == 'prepare':
            if acknowledged and self.status.ready and not self.status.armed:
                self.phase, self.ack_sequence = 'arming', None
            else:
                msg = self.packet(False, zero, epoch_ms)
                if self.ack_sequence is None:
                    self.ack_sequence = msg.sequence
                return msg
        if self.phase == 'arming':
            if acknowledged and self.status.armed:
                self.phase = 'active'
            else:
                msg = self.packet(True, zero, epoch_ms)
                if self.ack_sequence is None:
                    self.ack_sequence = msg.sequence
                return msg
        if self.phase == 'active' and self.status.armed:
            return self.packet(True, self.targets, epoch_ms)
        self.invalidate()
        return self.packet(False, zero, epoch_ms)


class Stm32Teleop(Node):
    def __init__(self):
        super().__init__('stm32_teleop')
        self.axes = [param(self, 'linear_axis', 1), param(self, 'twist_axis', 2),
                     param(self, 'arm_axis', 3)]
        self.axis_signs = [param(self, 'linear_sign', 1.0), param(self, 'twist_sign', 1.0),
                           param(self, 'arm_sign', 1.0)]
        self.deadzones = [param(self, 'drive_deadzone', 0.08), param(self, 'twist_deadzone', 0.10),
                          param(self, 'arm_deadzone', 0.20)]
        self.left, self.right = list(param(self, 'left_channels', [0, 1])), list(param(self, 'right_channels', [2, 3]))
        self.signs = list(param(self, 'motor_signs', [1, 1, 1, 1, 1]))
        self.drive_limit, self.arm_limit = param(self, 'drive_limit', 0.20), param(self, 'arm_limit', 0.15)
        self.deadman = param(self, 'deadman_button', 1)
        self.dry_run = param(self, 'dry_run', True)
        self.device = param(self, 'joystick_device',
                            '/dev/input/by-id/usb-Logitech_Extreme_3D_pro_00000000002A-joystick')
        timeout, status_timeout = param(self, 'input_timeout', 0.25), param(self, 'status_timeout', 0.15)
        if (sorted(self.left+self.right) != [0, 1, 2, 3] or len(self.left) != 2 or len(self.right) != 2
                or len(self.signs) != 5 or any(v not in (-1, 1) for v in self.signs)
                or len(set(self.axes)) != 3 or any(v < 0 for v in self.axes) or self.deadman < 0
                or any(v not in (-1.0, 1.0) for v in self.axis_signs)
                or any(not math.isfinite(v) or not 0 <= v < 1 for v in self.deadzones)
                or not all(math.isfinite(v) and v > 0 for v in (timeout, status_timeout))
                or timeout > 0.25 or status_timeout > 0.15
                or not math.isfinite(self.drive_limit) or not 0 < self.drive_limit <= 0.25
                or not math.isfinite(self.arm_limit) or not 0 < self.arm_limit <= 0.20):
            raise ValueError('Invalid channel/axis mapping, duty limit, deadzone or timeout')
        self.control = MotorSession(timeout, status_timeout)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        topic = '/motor/setpoints_preview' if self.dry_run else '/motor/setpoints'
        self.pub = self.create_publisher(MotorCommand, topic, qos)
        self.preview = self.create_publisher(Float64MultiArray, '/motor/joystick_targets', 1)
        self.create_subscription(MotorStatus, '/motor/status', self.on_status, qos)
        self.create_subscription(Joy, 'joy', self.on_joy, qos_profile_sensor_data)
        self.create_timer(0.05, self.update)
        self.last_state = None
        self.get_logger().info(
            f'{"DRY RUN" if self.dry_run else "LIVE"}: {topic}; axes forward/twist/arm={self.axes}; '
            f'hold button {self.deadman+1}. Center stick, twist and base lever before enabling.')

    def on_status(self, msg):
        self.control.on_status(msg, time.monotonic())

    def on_joy(self, msg):
        age = (self.get_clock().now().nanoseconds-
               (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if (not -0.05 <= age < self.control.input_timeout or len(msg.axes) <= max(self.axes)
                or len(msg.buttons) <= self.deadman
                or not all(math.isfinite(msg.axes[i]) for i in self.axes)):
            self.control.joy_at = float('-inf')
            self.control.invalidate()
            return
        values = [sign*max(-1.0, min(1.0, msg.axes[index]))
                  for index, sign in zip(self.axes, self.axis_signs)]
        neutral = all(abs(v) <= deadzone for v, deadzone in zip(values, self.deadzones))
        values = [0.0 if abs(v) <= dz else math.copysign((abs(v)-dz)/(1.0-dz), v)
                  for v, dz in zip(values, self.deadzones)]
        targets = mix_targets(*values, self.left, self.right, self.signs, self.drive_limit, self.arm_limit)
        self.control.on_joy(targets, bool(msg.buttons[self.deadman]), neutral,
                            time.monotonic()-max(0.0, age))

    def update(self):
        now = time.monotonic()
        if self.device and not Path(self.device).exists():
            self.control.joy_at = float('-inf')
            self.control.invalidate()
        fresh = now-self.control.joy_at < self.control.input_timeout
        demand = self.control.targets if fresh and self.control.held else [0.0]*5
        self.preview.publish(Float64MultiArray(data=demand))
        command = self.control.step(now, time.time_ns()//1_000_000)
        if command is not None:
            self.pub.publish(command)
        state = (self.control.phase, self.control.locked,
                 now-self.control.status_at < self.control.status_timeout,
                 self.control.status.hardware_ok if self.control.status else False)
        if state != self.last_state:
            self.get_logger().info(f'Control state: phase={state[0]}, neutral/release required={state[1]}, '
                                   f'fresh STM status={state[2]}, STM motor profile ready={state[3]}')
            self.last_state = state

    def destroy_node(self):
        if self.context.ok() and self.control.session and self.control.sequence < 0xffffffff:
            self.pub.publish(self.control.packet(False, [0.0]*5, time.time_ns()//1_000_000))
        return super().destroy_node()


def main(args=None):
    run(Stm32Teleop, args)
