"""Tests for the sensor models (complete) and the estimators (ComplementaryFilter is a TODO)."""
import numpy as np
import pytest

from balancebot import RobotParams
from balancebot.dynamics import forward_dynamics
from balancebot.estimators import ComplementaryFilter, EncoderOdometry, accel_angle
from balancebot.sensors import Sensors, SensorParams

P = RobotParams()
NOISELESS = SensorParams(acc_noise=0.0, gyro_noise=0.0, gyro_bias=0.0)
DT = 5e-3


# ---------------------------------------------------------------- sensors (complete)
@pytest.mark.parametrize("deg", [-30, -5, 0, 5, 30])
def test_accelerometer_at_rest_measures_gravity(deg):
    th = np.radians(deg)
    s = np.array([0.0, th, 0.0, 0.0])
    _, af, au = Sensors(P, NOISELESS).imu(s, np.zeros(4))     # held still: no acceleration
    assert af == pytest.approx(-P.g * np.sin(th))
    assert au == pytest.approx(P.g * np.cos(th))
    assert accel_angle(af, au) == pytest.approx(th)


def test_accelerometer_is_disturbed_by_acceleration():
    """While falling, the accelerometer angle is wrong: it is not only gravity."""
    s = np.array([0.0, np.radians(10), 0.0, 0.0])
    _, af, au = Sensors(P, NOISELESS).imu(s, forward_dynamics(s, 0.0, P))
    assert abs(accel_angle(af, au) - s[1]) > np.radians(1)


def test_gyro_bias():
    sp = SensorParams(gyro_noise=0.0, gyro_bias=0.02)
    g, _, _ = Sensors(P, sp).imu(np.array([0.0, 0.0, 0.0, 0.5]), np.zeros(4))
    assert g == pytest.approx(0.52)


def test_encoder_measures_rotation_relative_to_body():
    sens = Sensors(P, NOISELESS)
    turn = 2 * np.pi * P.r
    assert sens.encoder(np.array([turn, 0.0, 0.0, 0.0])) == NOISELESS.counts_per_rev
    # body tilted, wheels still on the ground: the encoder sees -theta
    assert sens.encoder(np.array([0.0, np.pi, 0.0, 0.0])) == -NOISELESS.counts_per_rev // 2


def test_odometry_corrects_with_theta():
    odo = EncoderOdometry(P, NOISELESS, DT)
    sens = Sensors(P, NOISELESS)
    odo.update(sens.encoder(np.zeros(4)), 0.0)
    s = np.array([0.0, np.radians(20), 0.0, 0.0])            # tilted, axle has not moved
    x, _ = odo.update(sens.encoder(s), s[1])
    assert abs(x) < 2 * np.pi * P.r / NOISELESS.counts_per_rev   # within one count


# ---------------------------------------------------------------- complementary filter (TODO)
def readings(theta, theta_dot=0.0, bias=0.0):
    return theta_dot + bias, -P.g * np.sin(theta), P.g * np.cos(theta)


def test_cf_alpha_zero_is_accelerometer():
    cf = ComplementaryFilter(0.0, DT)
    th, _ = cf.update(*readings(0.2, theta_dot=1.0))
    assert th == pytest.approx(0.2)


def test_cf_alpha_one_is_gyro_integration():
    cf = ComplementaryFilter(1.0, DT, theta0=0.1)
    th, _ = cf.update(*readings(0.5, theta_dot=2.0))
    assert th == pytest.approx(0.1 + 2.0 * DT)


def test_cf_converges_to_accelerometer_when_still():
    cf = ComplementaryFilter(0.98, DT)
    for _ in range(2000):
        th, _ = cf.update(*readings(0.3))
    assert th == pytest.approx(0.3, abs=1e-6)


def test_cf_gyro_bias_gives_bounded_offset():
    """A constant gyro bias b gives a constant error ~ b * T, not an endless drift."""
    alpha, b = 0.98, 0.02
    cf = ComplementaryFilter(alpha, DT)
    errs = [cf.update(*readings(0.0, bias=b))[0] for _ in range(4000)]
    T = alpha * DT / (1 - alpha)
    assert errs[-1] == pytest.approx(b * T, rel=0.05)
    assert abs(errs[-1] - errs[-1000]) < 1e-6


def test_cf_rate_output_is_gyro():
    cf = ComplementaryFilter(0.98, DT)
    _, thd = cf.update(*readings(0.0, theta_dot=0.7))
    assert thd == pytest.approx(0.7)
