"""Experiment 03 - PID on the tilt angle.

The robot starts at rest tilted by THETA0 and a PID on theta alone tries to keep
it upright. Outputs in runs/03_pid/: control.png (theta, x, tau) and pid.gif.

Fill in the gains below (and the PID class in balancebot/controllers.py).
Run from the repository root:  python experiments/03_pid.py
"""
from pathlib import Path

import numpy as np

from balancebot import RobotParams, viz
from balancebot.controllers import PID
from balancebot.simulate import simulate

OUT = Path("runs/03_pid")
p = RobotParams()

# ---------------------------------------------------------------- settings
KP, KI, KD = 0.61, 0.026, 0.01     # TODO: choose the gains
TAU_MAX = None                    # total torque limit [N m]; None = unlimited (no motor model yet)
THETA0 = np.radians(5.0)
DT, T_FINAL = 5e-3, 5.0

# ---------------------------------------------------------------- run
pid = PID(KP, KI, KD, DT, tau_max=TAU_MAX)
fallen = lambda t, s: abs(s[1]) >= np.pi / 2
res = simulate(p, [0.0, THETA0, 0.0, 0.0], T_FINAL, DT, controller=pid, stop=fallen)

if res["t"][-1] < T_FINAL - DT / 2:
    print(f"fell after {res['t'][-1]:.2f} s")
else:
    th, x = res["s"][:, 1], res["s"][:, 0]
    print(f"final theta = {np.degrees(th[-1]):.2f} deg, max |theta| = {np.degrees(np.abs(th).max()):.1f} deg")
    print(f"final x = {100 * x[-1]:.1f} cm, final x_dot = {res['s'][-1, 2]:.3f} m/s")
    print(f"max |tau| = {np.abs(res['tau']).max():.3f} N m")

viz.plot_control(res, title=f"PID  kp={KP}, ki={KI}, kd={KD}", path=OUT / "control.png", tau_max=TAU_MAX)
viz.animate(res, p, path=OUT / "pid.gif")
print(f"results saved in {OUT}/")
