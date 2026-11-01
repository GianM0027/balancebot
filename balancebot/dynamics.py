"""Planar dynamics of the robot (report, Section 1).

    M(q) q_dd + h(q, q_d) = B tau,      q = [x, theta]

TO DO (you): implement the five functions below. Use only NumPy.
Run `pytest tests/test_dynamics.py -v` to check them.

All functions are pure: they take arrays and parameters and return new arrays,
without hidden state. This keeps them easy to test and easy to port to JAX later.
"""
import numpy as np
from .params import RobotParams


def mass_matrix(q: np.ndarray, p: RobotParams) -> np.ndarray:
    """Mass matrix M(q), shape (2, 2).

    Args:
        q: generalised coordinates [x, theta].
        p: robot parameters.

    Returns:
        [[a, b],
         [b, c]]  with a, b, c as defined in the report (Eq. abc).
    """
    theta = q[1]
    a = p.m_w + (p.I_w)*(1/p.r**2) + p.m_b
    b = p.m_b*p.l*np.cos(theta)
    c = p.m_b*p.l**2 + p.I_b

    return np.array([[a, b],
                     [b, c]])


def bias_vector(q: np.ndarray, qd: np.ndarray, p: RobotParams) -> np.ndarray:
    """Vector h(q, q_d), shape (2,): centrifugal and gravity terms.

    Args:
        q:  [x, theta].
        qd: [x_dot, theta_dot].
        p:  robot parameters.
    """
    theta = q[1]
    theta_dot = qd[1]

    return np.array([-p.m_b*p.l*np.sin(theta)*theta_dot**2,
                     -p.m_b*p.g*p.l*np.sin(theta)])



def input_vector(p: RobotParams) -> np.ndarray:
    """Input vector B, shape (2,): how the motor torque tau enters each equation."""
    return np.array([1/p.r, -1.0])


def forward_dynamics(s: np.ndarray, tau: float, p: RobotParams) -> np.ndarray:
    """Time derivative of the state, s_dot = f(s, tau), shape (4,).

    Args:
        s:   state [x, theta, x_dot, theta_dot].
        tau: motor torque [N m].
        p:   robot parameters.

    Returns:
        [x_dot, theta_dot, x_ddot, theta_ddot], where the accelerations solve
        M(q) q_dd = B tau - h(q, q_d).

    Hint: solve the linear system with np.linalg.solve instead of computing
    the inverse of M explicitly (faster and numerically better).
    """
    q, qd = s[:2], s[2:]
    M = mass_matrix(q, p)
    rhs = input_vector(p) * tau - bias_vector(q, qd, p)

    qdd = np.linalg.solve(M, rhs)
    return np.concatenate([qd, qdd])


def energy(s: np.ndarray, p: RobotParams) -> tuple[float, float]:
    """Kinetic and potential energy (T, V) of state s, in joules.
    Args:
        s:   state [x, theta, x_dot, theta_dot].
        p:   robot parameters.

    T = 1/2 q_d^T M(q) q_d,   V = m_b g l cos(theta)   (zero at the axle).
    """
    q, qd = s[:2], s[2:]

    T = 0.5 * qd @ mass_matrix(q, p) @ qd
    V = p.m_b * p.g * p.l * np.cos(q[1])

    return float(T), float(V)
