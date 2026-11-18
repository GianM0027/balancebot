"""DC gear-motor model (both wheel motors together).

Standard steady-state model of a brushed DC motor with gearbox, written with the
quantities found on the datasheet. For ONE motor, the torque at the wheel is

    tau_1 = tau_stall * (V_eff / V_nom) - (tau_stall / omega_0) * omega_rel

    V_eff      voltage applied by the driver, clipped to [-V_batt, V_batt] and
               reduced by the dead zone (gearbox static friction)
    omega_rel  speed of the wheel relative to the motor housing, which is fixed
               to the body:  omega_rel = phi_dot - theta_dot = x_dot / r - theta_dot

The first term is the torque at stall; the second is the effect of the back-EMF:
the torque falls linearly with speed and reaches zero at the no-load speed.
The winding inductance is neglected (electrical time constant ~1 ms, much shorter
than the dynamics of the robot). The same voltage is applied to both motors
(planar model), so the total torque is n_motors * tau_1.

Note: V = 0 does not mean tau = 0. With the driver in "short brake" the motor
terminals are shorted and the back-EMF term brakes the wheel.
"""
from dataclasses import dataclass

import numpy as np

from .params import RobotParams

KGCM = 0.0980665  # 1 kg cm in N m


@dataclass(frozen=True)
class MotorParams:
    """Datasheet values of one motor: 12 V, gearbox 21.3:1, 280 rpm (nominal)."""
    V_nom: float = 12.0                         # [V]    rated voltage
    tau_stall: float = 1.7 * KGCM               # [N m]  stall torque at V_nom (output shaft)
    omega_0: float = 280 * 2 * np.pi / 60       # [rad/s] no-load speed at V_nom (output shaft)
    i_stall: float = 1.3                        # [A]    stall current at V_nom
    i_noload: float = 0.06                      # [A]    no-load current at V_nom
    V_dead: float = 0.5                         # [V]    dead zone, estimate: (i_noload / i_stall) * V_nom
    V_batt: float = 11.1                        # [V]    battery voltage (10.8 empty ... 12.6 full)
    n_motors: int = 2


def relative_speed(s: np.ndarray, p: RobotParams) -> float:
    """Wheel speed relative to the body [rad/s]:  phi_dot - theta_dot.

    Args:
        s: state [x, theta, x_dot, theta_dot].
        p: robot parameters (wheel radius).
    """
    return s[2] / p.r - s[3]


def effective_voltage(V: float, m: MotorParams) -> float:
    """Voltage that actually drives the motor: battery limit and dead zone.

    Args:
        V: command voltage [V].
        m: motor parameters (V_batt, V_dead).

    Returns:
        V clipped to [-V_batt, V_batt] and reduced by V_dead in magnitude [V].
    """
    V = float(np.clip(V, -m.V_batt, m.V_batt))
    return float(np.sign(V) * max(abs(V) - m.V_dead, 0.0))


def torque(V: float, s: np.ndarray, p: RobotParams, m: MotorParams) -> float:
    """Total wheel torque [N m] produced by the motors for the command voltage V.

    Args:
        V: command voltage, the same for both motors [V].
        s: state [x, theta, x_dot, theta_dot] (the torque depends on the wheel speed).
        p: robot parameters.
        m: motor parameters.
    """
    w = relative_speed(s, p)
    tau_1 = m.tau_stall * effective_voltage(V, m) / m.V_nom - m.tau_stall / m.omega_0 * w
    return m.n_motors * tau_1


def current(V: float, s: np.ndarray, p: RobotParams, m: MotorParams) -> float:
    """Current drawn by ONE motor [A] (sign = direction), proportional to its torque.

    Args:
        V: command voltage [V].
        s: state [x, theta, x_dot, theta_dot].
        p: robot parameters.
        m: motor parameters.
    """
    w = relative_speed(s, p)
    return m.i_stall * (effective_voltage(V, m) / m.V_nom - w / m.omega_0)


def voltage_for_torque(tau: float, s: np.ndarray, p: RobotParams, m: MotorParams) -> float:
    """Voltage that produces the total torque tau at the current speed (inverse model).

    Compensates the back-EMF and the dead zone; the result is clipped to the battery.

    Args:
        tau: desired total wheel torque [N m].
        s:   state [x, theta, x_dot, theta_dot].
        p:   robot parameters.
        m:   motor parameters.

    Returns:
        Command voltage [V], in [-V_batt, V_batt].
    """
    w = relative_speed(s, p)
    V = m.V_nom * (tau / (m.n_motors * m.tau_stall) + w / m.omega_0)
    if V != 0.0:
        V += np.sign(V) * m.V_dead
    return float(np.clip(V, -m.V_batt, m.V_batt))


def torque_limits(s: np.ndarray, p: RobotParams, m: MotorParams):
    """Smallest and largest total torque available at the current speed.

    Args:
        s: state [x, theta, x_dot, theta_dot].
        p: robot parameters.
        m: motor parameters.

    Returns:
        (tau_min, tau_max) [N m], obtained with V = -V_batt and V = +V_batt.
    """
    return torque(-m.V_batt, s, p, m), torque(m.V_batt, s, p, m)
