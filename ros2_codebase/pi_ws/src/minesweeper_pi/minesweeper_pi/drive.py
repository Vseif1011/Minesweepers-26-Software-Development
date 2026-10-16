"""Differential drive math and the bounded, line-oriented Mega wire protocol."""
import math


def wheel_speeds(linear, angular, track_width=0.55, max_speed=1.0):
    if track_width <= 0 or max_speed <= 0:
        raise ValueError('track_width and max_speed must be positive')
    if not all(math.isfinite(v) for v in (linear, angular)):
        return 0.0, 0.0
    left = linear - angular * track_width / 2
    right = linear + angular * track_width / 2
    if not all(math.isfinite(v) for v in (left, right)):
        return 0.0, 0.0
    scale = max(1.0, abs(left) / max_speed, abs(right) / max_speed)
    return left / scale, right / scale


def motor_duties(left, right, max_speed=1.0):
    # Physical output ordering/signs from the old Pi/Mega code: FR, FL, RL, RR.
    # ROS positive angular velocity now means a counterclockwise turn.
    return [-100 * right / max_speed, 100 * left / max_speed,
            100 * left / max_speed, -100 * right / max_speed]


def parse_telemetry(line):
    """T,left_int32,right_int32,metal_0_or_1; no evaluation or unbounded frames."""
    if len(line) > 96:
        raise ValueError('oversized telemetry')
    parts = line.strip().split(',')
    if len(parts) != 4 or parts[0] != 'T':
        raise ValueError('expected T,left,right,metal')
    left, right, metal = (int(v) for v in parts[1:])
    if not (-2**31 <= left < 2**31 and -2**31 <= right < 2**31 and metal in (0, 1)):
        raise ValueError('invalid telemetry range')
    return left, right, bool(metal)


def encode_command(linear, angular, controller):
    if controller not in (0, 5, 6, 7, 8, 9, 10, 11):
        raise ValueError('invalid controller command')
    if not all(math.isfinite(v) for v in (linear, angular)):
        linear = angular = 0.0
    return f'C,{linear:.5f},{angular:.5f},{controller}\n'.encode('ascii')
