"""Simulation loop.

At every step the controller reads the current state and returns a torque,
which is then held constant during the step (zero-order hold), as a digital
controller on the microcontroller would do.
"""
from typing import Callable, Optional

import numpy as np

from . import dynamics
from .integrators import INTEGRATORS
from .params import RobotParams

Controller = Callable[[float, np.ndarray], float]
StopCondition = Callable[[float, np.ndarray], bool]


def zero_controller(t: float, s: np.ndarray) -> float:
    """Motor off."""
    return 0.0


def simulate(
    p: RobotParams,
    s0,
    t_final: float,
    dt: float,
    controller: Controller = zero_controller,
    integrator: str = "rk4",
    stop: Optional[StopCondition] = None,
) -> dict:
    """Simulate the robot from initial state s0 for t_final seconds.

    Args:
        p:          robot parameters.
        s0:         initial state [x, theta, x_dot, theta_dot].
        t_final:    duration [s].
        dt:         integration step [s].
        controller: function (t, s) -> tau.
        integrator: "euler" or "rk4".
        stop:       optional function (t, s) -> bool that ends the simulation early.

    Returns:
        dict with arrays
            t   (N,)    time
            s   (N, 4)  state
            tau (N,)    applied torque (last entry repeated)
            T, V (N,)   kinetic and potential energy
    """
    step = INTEGRATORS[integrator]
    n_steps = int(round(t_final / dt))

    t_log = np.zeros(n_steps + 1)
    s_log = np.zeros((n_steps + 1, 4))
    tau_log = np.zeros(n_steps + 1)
    s_log[0] = np.asarray(s0, dtype=float)

    n = n_steps
    for k in range(n_steps):
        t, s = t_log[k], s_log[k]
        tau = float(controller(t, s))

        def f(t_, s_, tau=tau):
            return dynamics.forward_dynamics(s_, tau, p)

        s_log[k + 1] = step(f, t, s, dt)
        t_log[k + 1] = t + dt
        tau_log[k] = tau
        if stop is not None and stop(t_log[k + 1], s_log[k + 1]):
            n = k + 1
            break

    t_log, s_log, tau_log = t_log[: n + 1], s_log[: n + 1], tau_log[: n + 1]
    if n > 0:
        tau_log[-1] = tau_log[-2]
    energies = np.array([dynamics.energy(s, p) for s in s_log])
    return {"t": t_log, "s": s_log, "tau": tau_log, "T": energies[:, 0], "V": energies[:, 1]}
