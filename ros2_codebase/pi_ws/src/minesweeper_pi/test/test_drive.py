import math

import numpy as np
import pytest

from minesweeper_pi.drive import encode_command, motor_duties, parse_telemetry, wheel_speeds
from minesweeper_pi.imu import madgwick


def test_ros_positive_rotation_and_ratio_preserving_saturation():
    left, right = wheel_speeds(0.5, 1.0)
    assert right > left > 0
    left, right = wheel_speeds(2.0, 2.0)
    assert right == pytest.approx(1.0)
    assert left/right == pytest.approx((2.0-0.55)/(2.0+0.55))


def test_duties_use_percent_and_legacy_wiring_signs():
    assert motor_duties(0.5, 1.0) == [-100., 50., 50., -100.]


@pytest.mark.parametrize('v,w', [(math.nan, 0), (0, math.inf), (-math.inf, 1)])
def test_nonfinite_drive_stops(v, w):
    assert wheel_speeds(v, w) == (0., 0.)
    assert encode_command(v, w, 9) == b'C,0.00000,0.00000,9\n'


@pytest.mark.parametrize('line', ['T,1,2,2', 'T,2147483648,0,0', 'T,1,x,0', 'bad', 'x'*100])
def test_bad_serial_frames_rejected(line):
    with pytest.raises(ValueError):
        parse_telemetry(line)


def test_full_width_serial_roundtrip():
    assert parse_telemetry('T,-2147483648,2147483647,1\r\n') == (-2147483648, 2147483647, True)
    assert encode_command(0.2, -0.1, 11) == b'C,0.20000,-0.10000,11\n'
    with pytest.raises(ValueError):
        encode_command(0., 0., 12)


def test_level_stationary_imu_does_not_divide_by_zero():
    q = np.array([1., 0., 0., 0.])
    for _ in range(500):
        q = madgwick(q, [0., 0., 0.], [0., 0., 9.80665], 0.01)
    np.testing.assert_allclose(q, [1., 0., 0., 0.])


def test_imu_integrates_known_turn_and_handles_zero_acceleration():
    q = np.array([1., 0., 0., 0.])
    for _ in range(100):
        q = madgwick(q, [0., 0., math.pi/2], [0., 0., 0.], 0.01)
    assert np.linalg.norm(q) == pytest.approx(1.)
    assert 2*math.atan2(q[3], q[0]) == pytest.approx(math.pi/2, abs=0.001)
