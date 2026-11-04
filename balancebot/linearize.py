"""Linearisation of the model around the upright equilibrium.

    s_dot ~= A s + B tau,   s = [x, theta, x_dot, theta_dot],   equilibrium s = 0, tau = 0

The analytic matrices come from the first-order Taylor expansion of
M(q) q_dd + h(q, q_d) = B tau  (sin theta ~ theta, cos theta ~ 1, theta * theta_dot^2 ~ 0).
"""
from typing import Callable, Tuple

import numpy as np

from .params import RobotParams


def linearize(p: RobotParams) -> Tuple[np.ndarray, np.ndarray]:
    """Analytic A (4x4) and B (4x1) at the upright equilibrium."""
    a = p.m_w + p.I_w / p.r**2 + p.m_b
    b0 = p.m_b * p.l
    c = p.m_b * p.l**2 + p.I_b
    D = a * c - b0**2                      # det of the mass matrix at theta = 0
    mgl = p.m_b * p.g * p.l
    A = np.array([[0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, 0.0, 1.0],
                  [0.0, -b0 * mgl / D, 0.0, 0.0],
                  [0.0, a * mgl / D, 0.0, 0.0]])
    B = np.array([[0.0], [0.0], [(c / p.r + b0) / D], [-(a + b0 / p.r) / D]])
    return A, B


def linearize_numerical(f: Callable[[np.ndarray, float], np.ndarray],
                        s_eq: np.ndarray, tau_eq: float = 0.0,
                        eps: float = 1e-6) -> Tuple[np.ndarray, np.ndarray]:
    """Jacobians of s_dot = f(s, tau) at (s_eq, tau_eq) by central differences."""
    s_eq = np.asarray(s_eq, dtype=float)
    A = np.column_stack([(f(s_eq + eps * e, tau_eq) - f(s_eq - eps * e, tau_eq)) / (2 * eps)
                         for e in np.eye(len(s_eq))])
    B = ((f(s_eq, tau_eq + eps) - f(s_eq, tau_eq - eps)) / (2 * eps)).reshape(-1, 1)
    return A, B


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """[B, AB, A^2 B, ..., A^(n-1) B]."""
    return np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(A.shape[0])])
