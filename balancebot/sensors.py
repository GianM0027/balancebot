"""Sensor models: MPU-6050 (accelerometer + gyroscope) and wheel encoders.

The IMU is mounted on the body at distance d_imu from the axle, along the body
axis. Body axes: "up" along the body (from axle to CoM), "fwd" perpendicular to
it, pointing towards +x when the body is upright.

    accelerometer: specific force (acceleration minus gravity) of the mounting
                   point, projected on the body axes, plus noise
    gyroscope:     theta_dot + bias + noise
    encoder:       integer count of the wheel rotation RELATIVE TO THE BODY
                   (the encoder sits on the motor, which is fixed to the body)

Noise and bias values are measured on the real MPU-6050 (60 s still, 200 Hz,
DLPF 44 Hz, see data/imu_still.csv); the gyro bias is the residual left after
a 2 s start-up calibration.
"""
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .params import RobotParams


@dataclass(frozen=True)
class SensorParams:
    d_imu: float = 0.10                 # [m]     axle -> IMU, along the body
    acc_noise: float = 0.015            # [m/s^2] std per sample, MEASURED (data/imu_still.csv, 200 Hz, DLPF 44 Hz)
    gyro_noise: float = np.radians(0.04)  # [rad/s] std per sample, MEASURED (same recording)
    gyro_bias: float = np.radians(0.02)   # [rad/s] residual after a 2 s start-up calibration, MEASURED
                                          #         (raw bias before calibration: about -1.5 deg/s on x and y)
    counts_per_rev: int = 937            # encoder counts per wheel turn: 11 pulses x 21.3 gearbox x 4 (quadrature)
    seed: Optional[int] = 0


class Sensors:
    """Generates sensor readings from the true state. Keeps the random generator."""

    def __init__(self, p: RobotParams, sp: SensorParams = SensorParams()):
        """
        Args:
            p:  robot parameters (g, wheel radius).
            sp: sensor parameters (mounting, noise, bias, encoder resolution).
        """
        self.p, self.sp = p, sp
        self.rng = np.random.default_rng(sp.seed)

    def imu(self, s: np.ndarray, s_dot: np.ndarray):
        """IMU readings for state s with time derivative s_dot.

        Args:
            s:     true state [x, theta, x_dot, theta_dot].
            s_dot: its time derivative [x_dot, theta_dot, x_ddot, theta_ddot]
                   (the accelerometer also feels the accelerations).

        Returns:
            (gyro [rad/s], acc_fwd [m/s^2], acc_up [m/s^2]), with noise and bias.
        """
        x, th, xd, thd = s
        xdd, thdd = s_dot[2], s_dot[3]
        d, g = self.sp.d_imu, self.p.g
        # acceleration of the IMU point  p = (x + d sin(theta), d cos(theta))
        ax = xdd + d * (thdd * np.cos(th) - thd**2 * np.sin(th))
        az = d * (-thdd * np.sin(th) - thd**2 * np.cos(th))
        fx, fz = ax, az + g                                  # specific force = a - g_vec, g_vec = (0, -g)
        acc_fwd = fx * np.cos(th) - fz * np.sin(th)          # projection on the body axes
        acc_up = fx * np.sin(th) + fz * np.cos(th)
        n = self.rng.normal(size=3)
        gyro = thd + self.sp.gyro_bias + self.sp.gyro_noise * n[0]
        return gyro, acc_fwd + self.sp.acc_noise * n[1], acc_up + self.sp.acc_noise * n[2]

    def encoder(self, s: np.ndarray) -> int:
        """Encoder count: wheel angle relative to the body, quantised.

        Args:
            s: true state [x, theta, x_dot, theta_dot].

        Returns:
            Integer count (counts_per_rev per wheel turn relative to the body).
        """
        phi_rel = s[0] / self.p.r - s[1]
        return int(np.floor(phi_rel * self.sp.counts_per_rev / (2 * np.pi)))
