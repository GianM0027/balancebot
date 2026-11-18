"""Simulation loop.

At every step the controller reads the current state and returns its command,
which is then held constant during the step (zero-order hold), as a digital
controller on the microcontroller would do.

Without a motor model the command is the total wheel torque tau. With a motor
model (motor=MotorParams()) the command is the motor voltage: the voltage is
held during the step, while the torque it produces is recomputed inside the
step from the current speed (back-EMF).
"""
from typing import Callable, Optional

import numpy as np

from . import dynamics, motor as motor_model
from .integrators import INTEGRATORS
from .params import RobotParams

Controller = Callable[[float, np.ndarray], float]
StopCondition = Callable[[float, np.ndarray], bool]


def zero_controller(t: float, s: np.ndarray) -> float:
    """Motor off: always returns 0.

    Args:
        t: current time [s] (unused).
        s: current state [x, theta, x_dot, theta_dot] (unused).

    Returns:
        0.0
    """
    return 0.0


def simulate(
    p: RobotParams,
    s0,
    t_final: float,
    dt: float,
    controller: Controller = zero_controller,
    integrator: str = "rk4",
    stop: Optional[StopCondition] = None,
    motor: Optional[motor_model.MotorParams] = None,
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
        motor:      optional motor parameters; if given, controller returns a voltage.

    Returns:
        dict with arrays
            t   (N,)    time
            s   (N, 4)  state
            tau (N,)    applied torque at the start of each step (last entry repeated)
            u, i (N,)   only with a motor: command voltage and current of one motor
            T, V (N,)   kinetic and potential energy
    """
    step = INTEGRATORS[integrator]
    n_steps = int(round(t_final / dt))

    t_log = np.zeros(n_steps + 1)
    s_log = np.zeros((n_steps + 1, 4))
    tau_log = np.zeros(n_steps + 1)
    u_log = np.zeros(n_steps + 1)
    i_log = np.zeros(n_steps + 1)
    s_log[0] = np.asarray(s0, dtype=float)

    n = n_steps
    for k in range(n_steps):
        t, s = t_log[k], s_log[k]
        cmd = float(controller(t, s))
        if motor is None:
            tau = cmd

            def f(t_, s_, tau=tau):
                """Right-hand side for the integrator, torque held constant during the step.

                Args:
                    t_:  time [s] (unused: the model does not depend on time).
                    s_:  state at which the integrator evaluates the dynamics.
                    tau: total wheel torque of this step [N m].
                """
                return dynamics.forward_dynamics(s_, tau, p)
        else:
            tau = motor_model.torque(cmd, s, p, motor)
            u_log[k], i_log[k] = cmd, motor_model.current(cmd, s, p, motor)

            def f(t_, s_, V=cmd):
                """Right-hand side for the integrator, voltage held constant during the step.

                The torque is recomputed from the voltage at every evaluation, because it
                depends on the wheel speed (back-EMF).

                Args:
                    t_: time [s] (unused: the model does not depend on time).
                    s_: state at which the integrator evaluates the dynamics.
                    V:  motor voltage of this step [V].
                """
                return dynamics.forward_dynamics(s_, motor_model.torque(V, s_, p, motor), p)

        s_log[k + 1] = step(f, t, s, dt)
        t_log[k + 1] = t + dt
        tau_log[k] = tau
        if stop is not None and stop(t_log[k + 1], s_log[k + 1]):
            n = k + 1
            break

    t_log, s_log, tau_log = t_log[: n + 1], s_log[: n + 1], tau_log[: n + 1]
    u_log, i_log = u_log[: n + 1], i_log[: n + 1]
    if n > 0:
        tau_log[-1], u_log[-1], i_log[-1] = tau_log[-2], u_log[-2], i_log[-2]
    energies = np.array([dynamics.energy(s, p) for s in s_log])
    res = {"t": t_log, "s": s_log, "tau": tau_log, "T": energies[:, 0], "V": energies[:, 1]}
    if motor is not None:
        res.update(u=u_log, i=i_log)
    return res
