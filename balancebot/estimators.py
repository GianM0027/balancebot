"""State estimation from the sensors of balancebot.sensors.

    ComplementaryFilter   theta and theta_dot from gyroscope + accelerometer
    EncoderOdometry       x and x_dot from the encoder count and theta         (complete)
    StateEstimator        both together -> estimated state [x, theta, x_dot, theta_dot]
"""
import numpy as np

from .params import RobotParams
from .sensors import SensorParams


def accel_angle(acc_fwd: float, acc_up: float) -> float:
    """Tilt from the accelerometer alone, assuming the only force felt is gravity.

    * by design:
    acc_fwd = -g sin(theta),  acc_up = g cos(theta)

    * we eliminate g by doing "- acc_fwd / acc_up"
    ==> - acc_fwd / acc_up = g sin(theta) / g cos(theta) = sin(theta) / cos(theta) = tan(theta)
    ==> theta = arctan(tan(theta)) = arctan(- acc_fwd / acc_up)

    Args:
        acc_fwd: accelerometer, body forward axis [m/s^2].
        acc_up:  accelerometer, body up axis [m/s^2].

    Returns:
        theta [rad].
    """
    theta = float(np.arctan2(-acc_fwd, acc_up))
    return theta


class ComplementaryFilter:
    """Fuse gyroscope and accelerometer into an estimate of theta.

        theta_hat <- alpha * (theta_hat + gyro * dt) + (1 - alpha) * theta_acc

    The gyro term is trusted over short times (high-pass), the accelerometer
    angle over long times (low-pass). alpha in (0, 1) sets the crossover:
    time constant  T = alpha * dt / (1 - alpha).

    Questions to answer while writing it:
      * what happens to theta_hat with alpha = 1? and with alpha = 0?
      * the gyro has a constant bias b: does it make theta_hat drift forever?
      * which value should theta_dot_hat return?
    """

    def __init__(self, alpha: float, dt: float, theta0: float = 0.0):
        """
        Args:
            alpha:  weight of the gyro-integrated angle, 0 < alpha < 1.
            dt:     time between two updates [s].
            theta0: initial estimate [rad].
        """
        self.alpha, self.dt = alpha, dt
        self.theta = theta0
        self.theta_dot = 0.0

    def update(self, gyro: float, acc_fwd: float, acc_up: float):
        """Process one IMU reading.

        Args:
            gyro:    measured theta_dot [rad/s] (with bias and noise).
            acc_fwd: accelerometer, body forward axis [m/s^2].
            acc_up:  accelerometer, body up axis [m/s^2].

        Returns:
            (theta_hat, theta_dot_hat) [rad, rad/s]
        """
        theta_acc = accel_angle(acc_fwd, acc_up)
        theta_pred = self.theta + gyro * self.dt            # prediction from the gyro
        self.theta = self.alpha * theta_pred + (1 - self.alpha) * theta_acc
        self.theta_dot = gyro                               # gyro is the best rate measurement we have
        return self.theta, self.theta_dot


class EncoderOdometry:
    """x and x_dot from the encoder count.

    The encoder measures the wheel angle relative to the body, phi - theta, so
    the absolute wheel angle needs the estimated tilt:  phi = phi_rel + theta.
    """

    def __init__(self, p: RobotParams, sp: SensorParams, dt: float):
        """
        Args:
            p:  robot parameters (wheel radius).
            sp: sensor parameters (encoder counts per wheel turn).
            dt: time between two updates [s].
        """
        self.p, self.sp, self.dt = p, sp, dt
        self.count0 = None
        self.x_prev = None
        self.x = 0.0
        self.x_dot = 0.0

    def update(self, count: int, theta_hat: float):
        """Process one encoder reading.

        Args:
            count:     encoder count (wheel angle relative to the body).
            theta_hat: current estimate of the tilt [rad], needed to get the absolute wheel angle.

        Returns:
            (x_hat [m], x_dot_hat [m/s]); x is measured from the position at the first call.
        """
        if self.count0 is None:
            self.count0 = count
        phi_rel = (count - self.count0) * 2 * np.pi / self.sp.counts_per_rev
        x = (phi_rel + theta_hat) * self.p.r
        if self.x_prev is not None:
            self.x_dot = 0.8 * self.x_dot + 0.2 * (x - self.x_prev) / self.dt  # first-order low-pass
        self.x_prev, self.x = x, x
        return self.x, self.x_dot


class StateEstimator:
    """Estimated state [x, theta, x_dot, theta_dot] from IMU and encoder readings."""

    def __init__(self, p: RobotParams, sp: SensorParams, dt: float, alpha: float, theta0: float = 0.0):
        """
        Args:
            p:      robot parameters.
            sp:     sensor parameters.
            dt:     time between two updates [s].
            alpha:  complementary-filter weight, 0 < alpha < 1.
            theta0: initial tilt estimate [rad].
        """
        self.att = ComplementaryFilter(alpha, dt, theta0)
        self.odo = EncoderOdometry(p, sp, dt)

    def update(self, gyro, acc_fwd, acc_up, count) -> np.ndarray:
        """Process one set of sensor readings.

        Args:
            gyro:    measured theta_dot [rad/s].
            acc_fwd: accelerometer, body forward axis [m/s^2].
            acc_up:  accelerometer, body up axis [m/s^2].
            count:   encoder count.

        Returns:
            Estimated state [x, theta, x_dot, theta_dot].
        """
        th, thd = self.att.update(gyro, acc_fwd, acc_up)
        x, xd = self.odo.update(count, th)
        return np.array([x, th, xd, thd])
