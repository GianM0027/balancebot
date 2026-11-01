"""Tests for balancebot.dynamics, based on the properties derived in the report.

Run:  pytest tests/test_dynamics.py -v
"""
import numpy as np
import pytest

from balancebot import RobotParams
from balancebot import dynamics as dyn

P = RobotParams()
THETAS = np.radians([-80, -30, -5, 0, 5, 30, 80])


# ---------------------------------------------------------------- shapes
def test_shapes():
    q, qd, s = np.zeros(2), np.zeros(2), np.zeros(4)
    assert dyn.mass_matrix(q, P).shape == (2, 2)
    assert dyn.bias_vector(q, qd, P).shape == (2,)
    assert dyn.input_vector(P).shape == (2,)
    assert dyn.forward_dynamics(s, 0.0, P).shape == (4,)


# ---------------------------------------------------------------- mass matrix
@pytest.mark.parametrize("theta", THETAS)
def test_mass_matrix_symmetric_positive_definite(theta):
    M = dyn.mass_matrix(np.array([0.3, theta]), P)
    assert np.allclose(M, M.T), "M must be symmetric"
    assert np.all(np.linalg.eigvalsh(M) > 0), "M must be positive definite"


def test_mass_matrix_independent_of_x():
    M1 = dyn.mass_matrix(np.array([0.0, 0.2]), P)
    M2 = dyn.mass_matrix(np.array([5.0, 0.2]), P)
    assert np.allclose(M1, M2), "M depends only on theta"


# ---------------------------------------------------------------- symbolic reference
@pytest.mark.parametrize("theta", THETAS)
def test_matches_symbolic_model(theta):
    sp = pytest.importorskip("sympy")
    from tools.verify_eom_sympy import symbolic_model

    mdl = symbolic_model()
    xs, ths, xds, thds = mdl["state"]
    subs = {sym: getattr(P, name) for name, sym in mdl["params"].items()}
    q, qd = np.array([0.1, theta]), np.array([0.4, -1.3])
    subs.update({xs: q[0], ths: q[1], xds: qd[0], thds: qd[1]})

    M_ref = np.array(mdl["M"].subs(subs).evalf(), dtype=float)
    h_ref = np.array(mdl["h"].subs(subs).evalf(), dtype=float).ravel()
    B_ref = np.array(mdl["B"].subs(subs).evalf(), dtype=float).ravel()

    assert np.allclose(dyn.mass_matrix(q, P), M_ref)
    assert np.allclose(dyn.bias_vector(q, qd, P), h_ref)
    assert np.allclose(dyn.input_vector(P), B_ref)


# ---------------------------------------------------------------- equilibrium and stability
def test_upright_at_rest_is_equilibrium():
    sd = dyn.forward_dynamics(np.zeros(4), 0.0, P)
    assert np.allclose(sd, 0.0)


def test_tilted_forward_falls_forward_and_wheels_roll_back():
    sd = dyn.forward_dynamics(np.array([0.0, np.radians(5), 0.0, 0.0]), 0.0, P)
    x_dd, th_dd = sd[2], sd[3]
    assert th_dd > 0, "a robot tilted towards +x must accelerate further towards +x"
    assert x_dd < 0, "while the body falls forward the wheels must roll backwards"


def test_motor_torque_pushes_forward_and_tilts_back():
    sd = dyn.forward_dynamics(np.zeros(4), 0.1, P)
    assert sd[2] > 0, "positive torque accelerates the robot towards +x"
    assert sd[3] < 0, "the reaction on the body tilts it backwards"


def test_forward_dynamics_kinematic_part():
    s = np.array([0.2, 0.1, 0.7, -0.4])
    sd = dyn.forward_dynamics(s, 0.0, P)
    assert np.allclose(sd[:2], s[2:]), "the first two entries of s_dot are the velocities"


# ---------------------------------------------------------------- energy
def test_energy_values():
    T, V = dyn.energy(np.zeros(4), P)
    assert T == pytest.approx(0.0)
    assert V == pytest.approx(P.m_b * P.g * P.l)
    T, V = dyn.energy(np.array([0.0, np.pi / 2, 0.0, 0.0]), P)
    assert V == pytest.approx(0.0, abs=1e-12)


def test_energy_consistent_with_mass_matrix():
    s = np.array([0.0, 0.3, 0.5, -1.2])
    T, _ = dyn.energy(s, P)
    M = dyn.mass_matrix(s[:2], P)
    assert T == pytest.approx(0.5 * s[2:] @ M @ s[2:])


def test_energy_conserved_without_input():
    """With tau = 0 and no friction, T + V must be constant (RK4, small step)."""
    from balancebot.simulate import simulate

    res = simulate(P, [0.0, np.radians(3), 0.0, 0.0], t_final=2.0, dt=1e-3, integrator="rk4")
    E = res["T"] + res["V"]
    assert np.max(np.abs(E - E[0])) / abs(E[0]) < 1e-6
