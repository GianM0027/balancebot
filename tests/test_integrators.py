"""Tests for balancebot.integrators on a problem with a known exact solution.

Harmonic oscillator  s = [position, velocity],  s_dot = [v, -omega^2 * position],
exact solution  position(t) = cos(omega t)  for s(0) = [1, 0].

The order of a method shows up in how the error shrinks when dt is halved:
first order (Euler) -> error / 2, fourth order (RK4) -> error / 16.

Run:  pytest tests/test_integrators.py -v
"""
import numpy as np
import pytest

from balancebot.integrators import euler_step, rk4_step

OMEGA = 2.0
T_END = 2.0


def f(t, s):
    return np.array([s[1], -OMEGA**2 * s[0]])


def final_error(step, dt):
    s, t = np.array([1.0, 0.0]), 0.0
    for _ in range(int(round(T_END / dt))):
        s = step(f, t, s, dt)
        t += dt
    return abs(s[0] - np.cos(OMEGA * T_END))


def test_euler_single_step():
    s = euler_step(f, 0.0, np.array([1.0, 0.0]), 0.1)
    assert np.allclose(s, [1.0, -0.4])


def test_rk4_exact_for_cubic():
    """RK4 integrates s_dot = t^3 exactly (polynomial of degree <= 4 in t)."""
    g = lambda t, s: np.array([t**3])
    s = rk4_step(g, 0.0, np.array([0.0]), 1.0)
    assert s[0] == pytest.approx(0.25)


def test_euler_is_first_order():
    ratio = final_error(euler_step, 1e-3) / final_error(euler_step, 5e-4)
    assert 1.8 < ratio < 2.2, f"error ratio {ratio:.2f}, expected ~2"


def test_rk4_is_fourth_order():
    ratio = final_error(rk4_step, 2e-2) / final_error(rk4_step, 1e-2)
    assert 14 < ratio < 18, f"error ratio {ratio:.2f}, expected ~16"


def test_rk4_much_more_accurate_than_euler():
    assert final_error(rk4_step, 1e-2) < 1e-3 * final_error(euler_step, 1e-2)
