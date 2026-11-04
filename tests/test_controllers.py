"""Tests for balancebot.controllers.

They check the structure of the controllers, not the choice of gains/weights,
and are expected to fail until the TODOs in controllers.py are written.
"""
import numpy as np
import pytest

from balancebot import RobotParams
from balancebot.controllers import LQR, PID, lqr_gain, saturate
from balancebot.dynamics import forward_dynamics
from balancebot.linearize import controllability_matrix, linearize, linearize_numerical
from balancebot.simulate import simulate

P = RobotParams()
A, B = linearize(P)
DT = 5e-3


def state(theta=0.0, theta_dot=0.0):
    return np.array([0.0, theta, 0.0, theta_dot])


# ---------------------------------------------------------------- linearisation (complete)
def test_linearization_matches_numerical():
    A_num, B_num = linearize_numerical(lambda s, tau: forward_dynamics(s, tau, P), np.zeros(4))
    assert np.allclose(A, A_num, atol=1e-6)
    assert np.allclose(B, B_num, atol=1e-6)


def test_controllable():
    assert np.linalg.matrix_rank(controllability_matrix(A, B)) == 4


def test_saturate():
    assert saturate(5.0, 1.0) == 1.0
    assert saturate(-5.0, 1.0) == -1.0
    assert saturate(0.3, 1.0) == 0.3
    assert saturate(5.0, None) == 5.0


# ---------------------------------------------------------------- PID
def test_pid_zero_at_reference():
    pid = PID(1.0, 1.0, 1.0, DT)
    assert pid(0.0, state()) == pytest.approx(0.0)


def test_pid_proportional_term_is_linear():
    pid = PID(3.0, 0.0, 0.0, DT)
    t1, t2 = pid(0.0, state(0.05)), pid(0.0, state(0.10))
    assert abs(t1) == pytest.approx(3.0 * 0.05)
    assert t2 == pytest.approx(2 * t1)


def test_pid_derivative_uses_theta_dot():
    pid = PID(0.0, 0.0, 0.5, DT)
    assert abs(pid(0.0, state(0.0, 2.0))) == pytest.approx(0.5 * 2.0)


def test_pid_integral_accumulates_and_resets():
    pid = PID(0.0, 2.0, 0.0, DT)
    taus = [pid(k * DT, state(0.1)) for k in range(3)]
    assert abs(taus[0]) == pytest.approx(2.0 * 0.1 * DT)
    assert abs(taus[2]) == pytest.approx(3 * abs(taus[0]))
    pid.reset()
    assert abs(pid(0.0, state(0.1))) == pytest.approx(abs(taus[0]))


def test_pid_restoring_sign():
    """A forward tilt must produce the torque that tilts the body back (theta_dd < 0)."""
    tau = PID(1.0, 0.0, 0.0, DT)(0.0, state(0.05))
    assert (B[3, 0] * tau) < 0


def test_pid_derivative_damps():
    """While the body rotates forward (theta_dot > 0) the D term must oppose the
    rotation: same direction of torque as the P term for a forward tilt."""
    tau_d = PID(0.0, 0.0, 1.0, DT)(0.0, state(0.0, 1.0))
    assert (B[3, 0] * tau_d) < 0


def test_pid_saturation():
    assert abs(PID(100.0, 0.0, 0.0, DT, tau_max=0.2)(0.0, state(0.5))) == pytest.approx(0.2)


# ---------------------------------------------------------------- LQR
Q, R = np.diag([1.0, 10.0, 1.0, 1.0]), np.array([[100.0]])


def test_lqr_gain_shape():
    assert lqr_gain(A, B, Q, R).shape == (1, 4)


def test_lqr_closed_loop_stable():
    K = lqr_gain(A, B, Q, R)
    assert np.all(np.linalg.eigvals(A - B @ K).real < 0)


def test_lqr_control_law():
    ctrl = LQR(A, B, Q, R)
    s = np.array([0.1, 0.05, -0.2, 0.3])
    assert ctrl(0.0, s) == pytest.approx(float(-(ctrl.K @ s)[0]))


def test_lqr_balances_nonlinear_model():
    res = simulate(P, state(np.radians(5)), 5.0, DT, controller=LQR(A, B, Q, R))
    assert np.all(np.abs(res["s"][:, 1]) < np.radians(20))
    assert np.max(np.abs(res["s"][-1])) < 1e-2
