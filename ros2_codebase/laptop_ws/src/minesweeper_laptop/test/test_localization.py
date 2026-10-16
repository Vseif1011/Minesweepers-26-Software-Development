import math

import numpy as np
import pytest

from minesweeper_laptop.ekf import ExtendedKalmanFilter
from minesweeper_laptop.odometry import WheelOdometry, normalize, tick_delta


def test_encoder_first_sample_is_baseline_then_one_meter_straight():
    odom = WheelOdometry()
    assert odom.update(10000, -10000, 0.1) == (0., 0., 0., 0.)
    v, w, _, _ = odom.update(13800, -7745, 1.)
    assert (odom.x, odom.y, odom.theta, v, w) == pytest.approx((1., 0., 0., 1., 0.))


def test_encoder_wrap_both_directions():
    assert tick_delta(-2**31+9, 2**31-1) == 10
    assert tick_delta(2**31-1, -2**31+9) == -10


def test_exact_quarter_circle():
    model = WheelOdometry(1000000., 1000000., 1.)
    model.update(0, 0, 1.)
    model.update(round(math.pi/4*1e6), round(3*math.pi/4*1e6), 1.)
    assert model.theta == pytest.approx(math.pi/2, abs=2e-6)
    assert model.x == pytest.approx(1., abs=2e-6)
    assert model.y == pytest.approx(1., abs=2e-6)


def test_zero_dt_does_not_consume_ticks():
    model = WheelOdometry()
    model.update(0, 0, 1.)
    model.update(3800, 2255, 0.)
    model.update(3800, 2255, 1.)
    assert model.x == pytest.approx(1.)


def test_ekf_wraps_both_heading_measurements():
    ekf = ExtendedKalmanFilter()
    ekf.x[2] = math.pi-0.01
    ekf.correct([-math.pi+0.01, -math.pi+0.02], [2, 2], [0.01, 0.01])
    assert abs(normalize(ekf.x[2]-math.pi)) < 0.03


def test_ekf_rejects_nonfinite_and_zero_variance():
    ekf = ExtendedKalmanFilter()
    assert not ekf.correct([float('nan')], [0], [1.])
    assert not ekf.correct([1.], [0], [0.])
    assert np.isfinite(ekf.x).all()


def test_ekf_covariance_remains_positive_and_tracks_motion():
    ekf = ExtendedKalmanFilter()
    for i in range(1, 301):
        ekf.predict(0.05)
        ekf.correct([i*0.01, 0., 0., 0.2, 0.], [0, 1, 2, 3, 4], [0.03]*5)
        ekf.correct([0., 0.], [2, 4], [0.1, 0.01])
    assert ekf.x[0] == pytest.approx(3., abs=0.03)
    np.testing.assert_allclose(ekf.p, ekf.p.T, atol=1e-12)
    assert np.linalg.eigvalsh(ekf.p).min() > 0
