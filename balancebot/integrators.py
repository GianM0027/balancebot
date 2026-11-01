"""Fixed-step numerical integrators for  s_dot = f(t, s).

TO DO (you): implement the two functions below.
Run `pytest tests/test_integrators.py -v` to check them.

Both take one step of length dt from (t, s) and return the new state.
f is any function with signature f(t, s) -> s_dot (NumPy arrays).
"""
from typing import Callable
import numpy as np

Dynamics = Callable[[float, np.ndarray], np.ndarray]


def euler_step(f: Dynamics, t: float, s: np.ndarray, dt: float) -> np.ndarray:
    """One step of the explicit (forward) Euler method.

        s_{k+1} = s_k + dt * f(t_k, s_k)

    First-order accurate: the global error scales as dt.
    """
    s_new = s + dt * f(t, s)
    return s_new


def rk4_step(f: Dynamics, t: float, s: np.ndarray, dt: float) -> np.ndarray:
    """One step of the classical fourth-order Runge-Kutta method.

        k1 = f(t,        s)
        k2 = f(t + dt/2, s + dt/2 * k1)
        k3 = f(t + dt/2, s + dt/2 * k2)
        k4 = f(t + dt,   s + dt   * k3)
        s_{k+1} = s_k + dt/6 * (k1 + 2k2 + 2k3 + k4)

    Fourth-order accurate: the global error scales as dt^4.
    """
    k1 = f(t, s)
    k2 = f(t + dt/2, s + dt/2 * k1)
    k3 = f(t + dt/2, s + dt/2 * k2)
    k4 = f(t + dt,   s + dt   * k3)

    s_new = s + (dt/6 * (k1 + 2*k2 + 2*k3 + k4))

    return s_new


INTEGRATORS = {"euler": euler_step, "rk4": rk4_step}
