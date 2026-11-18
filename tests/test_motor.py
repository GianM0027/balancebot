"""Tests for the DC motor model (balancebot.motor) and its use in simulate()."""
import numpy as np
import pytest

from balancebot import RobotParams
from balancebot.controllers import LQR, VoltageCommand
from balancebot.linearize import linearize
from balancebot.motor import (MotorParams, current, effective_voltage, relative_speed,
                              torque, voltage_for_torque)
from balancebot.simulate import simulate

P = RobotParams()
M = MotorParams(V_dead=0.0, V_batt=12.0)     # ideal motor at rated voltage


def state_with_wheel_speed(w_rel):
    """Body upright and still, wheels turning at w_rel relative to the body."""
    return np.array([0.0, 0.0, w_rel * P.r, 0.0])


def test_stall_torque_at_rated_voltage():
    assert torque(12.0, np.zeros(4), P, M) == pytest.approx(2 * M.tau_stall)


def test_zero_torque_at_no_load_speed():
    assert torque(12.0, state_with_wheel_speed(M.omega_0), P, M) == pytest.approx(0.0, abs=1e-12)


def test_torque_linear_in_speed():
    t0 = torque(12.0, state_with_wheel_speed(0.0), P, M)
    t_half = torque(12.0, state_with_wheel_speed(M.omega_0 / 2), P, M)
    assert t_half == pytest.approx(t0 / 2)


def test_relative_speed_uses_body_rotation():
    """Body rotating forward with the wheels fixed on the ground: the motor sees -theta_dot."""
    assert relative_speed(np.array([0.0, 0.0, 0.0, 2.0]), P) == pytest.approx(-2.0)


def test_zero_voltage_brakes():
    """With V = 0 the shorted motor opposes the relative motion (back-EMF)."""
    assert torque(0.0, state_with_wheel_speed(5.0), P, M) < 0


def test_battery_limit_and_dead_zone():
    m = MotorParams(V_dead=0.5, V_batt=11.1)
    assert effective_voltage(20.0, m) == pytest.approx(11.1 - 0.5)
    assert effective_voltage(-20.0, m) == pytest.approx(-(11.1 - 0.5))
    assert effective_voltage(0.3, m) == 0.0


def test_current_at_stall():
    assert current(12.0, np.zeros(4), P, M) == pytest.approx(M.i_stall)


@pytest.mark.parametrize("tau", [-0.2, -0.05, 0.05, 0.2])
def test_voltage_for_torque_is_inverse(tau):
    m = MotorParams()
    s = np.array([0.0, 0.05, 0.3, -0.5])
    assert torque(voltage_for_torque(tau, s, P, m), s, P, m) == pytest.approx(tau)


def test_simulate_with_zero_voltage_brakes_the_fall():
    """V = 0 from rest: zero torque at the start, then the back-EMF opposes the
    relative motion, so the body falls more slowly than with tau = 0."""
    s0 = [0.0, np.radians(3), 0.0, 0.0]
    a = simulate(P, s0, 0.2, 1e-3, motor=MotorParams())
    b = simulate(P, s0, 0.2, 1e-3)
    assert a["tau"][0] == pytest.approx(0.0)
    assert a["s"][-1, 1] < b["s"][-1, 1]


def test_lqr_balances_with_real_motors():
    A, B = linearize(P)
    m = MotorParams()
    ctrl = VoltageCommand(LQR(A, B, np.diag([1.0, 10.0, 1.0, 1.0]), np.array([[100.0]])), P, m)
    res = simulate(P, [0.0, np.radians(10), 0.0, 0.0], 5.0, 5e-3, controller=ctrl, motor=m)
    assert np.max(np.abs(res["s"][-1])) < 2e-2
    assert np.all(np.abs(res["u"]) <= m.V_batt + 1e-9)
