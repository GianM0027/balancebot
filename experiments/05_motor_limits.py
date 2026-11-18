"""Experiment 05 - PID and LQR driving the real motors (DC motor model).

The controllers still compute a torque; VoltageCommand converts it into a motor
voltage with the inverse motor model. The motor model then limits the torque
actually produced: battery voltage, back-EMF (torque falls with speed), dead zone.

Outputs in runs/05_motor/:
  response.png    theta, x, requested vs produced torque with the available range, voltage
  pid.gif, lqr.gif
  roa.png         largest recoverable initial tilt vs battery voltage

Run from the repository root:  python experiments/05_motor_limits.py
"""
from dataclasses import replace
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from balancebot import RobotParams, viz
from balancebot.controllers import LQR, PID, VoltageCommand
from balancebot.linearize import linearize
from balancebot.motor import MotorParams, torque_limits
from balancebot.simulate import simulate

OUT = Path("runs/05_motor")
OUT.mkdir(parents=True, exist_ok=True)
p, m = RobotParams(), MotorParams()
A, B = linearize(p)
DT, T_FINAL, THETA0 = 5e-3, 5.0, np.radians(10.0)
fallen = lambda t, s: abs(s[1]) >= np.pi / 2


def make(name, motor):
    """Fresh controller (the PID keeps an internal state).

    Args:
        name:  "PID" or "LQR".
        motor: motor parameters used by the inverse motor model.

    Returns:
        (voltage controller to pass to simulate, wrapped torque controller).
    """
    if name == "PID":
        tc = PID(0.61, 0.026, 0.01, DT)                                  # gains of experiment 03
    else:
        tc = LQR(A, B, np.diag([1.0, 10.0, 1.0, 1.0]), np.array([[100.0]]))  # weights of experiment 04
    return VoltageCommand(tc, p, motor), tc


# ---------------------------------------------------------------- 1. response from THETA0
fig, ax = plt.subplots(4, 2, figsize=(11, 9), sharex=True)
for col, name in enumerate(("PID", "LQR")):
    ctrl, tc = make(name, m)
    res = simulate(p, [0.0, THETA0, 0.0, 0.0], T_FINAL, DT, controller=ctrl, stop=fallen, motor=m)
    t, s = res["t"], res["s"]
    req = np.array([tc(tt, ss) for tt, ss in zip(t, s)]) if name == "LQR" else None
    lim = np.array([torque_limits(ss, p, m) for ss in s])
    ax[0, col].plot(t, np.degrees(s[:, 1])); ax[0, col].set_ylabel(r"$\theta$ [deg]")
    ax[1, col].plot(t, 100 * s[:, 0]); ax[1, col].set_ylabel(r"$x$ [cm]")
    ax[2, col].fill_between(t, lim[:, 0], lim[:, 1], color="0.9", label="available")
    if req is not None:
        ax[2, col].plot(t, req, "--", label="requested")
    ax[2, col].plot(t, res["tau"], label="produced"); ax[2, col].set_ylabel(r"$\tau$ [N m]")
    ax[2, col].legend(fontsize=8)
    ax[3, col].plot(t, res["u"]); ax[3, col].set_ylabel("V [V]"); ax[3, col].set_xlabel("t [s]")
    ax[0, col].set_title(name)
    print(f"{name}: final theta = {np.degrees(s[-1, 1]):.2f} deg, final x = {100 * s[-1, 0]:.1f} cm, "
          f"max |V| = {np.abs(res['u']).max():.2f} V, max |i| = {np.abs(res['i']).max():.2f} A per motor")
    viz.animate(res, p, path=OUT / f"{name.lower()}.gif")
for a in ax.flat:
    a.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / "response.png", dpi=130)


# ---------------------------------------------------------------- 2. region of attraction vs battery
def max_recoverable(name, motor, angles):
    """Largest initial tilt (from `angles`) recovered before the first failure.

    Args:
        name:   "PID" or "LQR".
        motor:  motor parameters (e.g. with a given battery voltage).
        angles: initial tilts to try, increasing [deg].

    Returns:
        Largest recovered tilt [deg] (0 if none).
    """
    best = 0
    for a in angles:
        ctrl, _ = make(name, motor)
        r = simulate(p, [0.0, np.radians(a), 0.0, 0.0], T_FINAL, DT, controller=ctrl, stop=fallen, motor=motor)
        if r["t"][-1] >= T_FINAL - DT / 2 and abs(r["s"][-1, 1]) < np.radians(1):
            best = a
        else:
            break
    return best


angles = np.arange(2, 62, 2)
volts = [10.8, 11.1, 12.0, 12.6]
fig, axr = plt.subplots(figsize=(6, 3.5))
for name in ("PID", "LQR"):
    roa = [max_recoverable(name, replace(m, V_batt=v), angles) for v in volts]
    print(f"{name}: recovers up to " + ", ".join(f"{a} deg at {v} V" for a, v in zip(roa, volts)))
    axr.plot(volts, roa, "o-", label=name)
axr.set_xlabel("battery voltage [V]")
axr.set_ylabel("max recoverable tilt [deg]")
axr.legend(); axr.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / "roa.png", dpi=130)
print(f"results saved in {OUT}/")
