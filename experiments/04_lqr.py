"""Experiment 04 - LQR on the full state.

1. LQR from THETA0: response on the nonlinear model (control.png, lqr.gif)
2. region of attraction: largest initial tilt from which the LQR still
   recovers, with and without a torque limit (roa.png)

Fill in Q and R below (and lqr_gain / LQR in balancebot/controllers.py).
Run from the repository root:  python experiments/04_lqr.py
"""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from balancebot import RobotParams, viz
from balancebot.controllers import LQR
from balancebot.linearize import linearize
from balancebot.simulate import simulate

OUT = Path("runs/04_lqr")
p = RobotParams()
A, B = linearize(p)

# ---------------------------------------------------------------- settings
Q = np.diag([1.0, 10.0, 1.0, 1.0])   # weights on [x, theta, x_dot, theta_dot]
R = np.array([[100.0]])              # weight on tau
TAU_MAX = 0.5       # total torque limit [N m]: placeholder until the motor model exists
THETA0 = np.radians(5.0)
DT, T_FINAL = 5e-3, 5.0
fallen = lambda t, s: abs(s[1]) >= np.pi / 2

# ---------------------------------------------------------------- 1. response
lqr = LQR(A, B, Q, R)
# The controller acts every DT seconds (zero-order hold), while the LQR is designed
# in continuous time: the fastest closed-loop eigenvalue must satisfy |lambda| * DT << 1,
# otherwise the sampled loop can become unstable even if A - B K is stable.
print("K =", np.array2string(lqr.K.ravel(), precision=3))
eig_cl = np.linalg.eigvals(A - B @ lqr.K)
print("closed-loop eigenvalues:", np.array2string(eig_cl, precision=2))
print(f"max |lambda| * DT = {np.abs(eig_cl).max() * DT:.2f}")

res = simulate(p, [0.0, THETA0, 0.0, 0.0], T_FINAL, DT, controller=lqr, stop=fallen)
print(f"max |theta| = {np.degrees(np.abs(res['s'][:, 1])).max():.1f} deg, "
      f"max |x| = {100 * np.abs(res['s'][:, 0]).max():.1f} cm, "
      f"max |tau| = {np.abs(res['tau']).max():.3f} N m")
viz.plot_control(res, title="LQR", path=OUT / "control.png")
viz.animate(res, p, path=OUT / "lqr.gif")


# ---------------------------------------------------------------- 2. region of attraction
def recovers(theta0: float, tau_max) -> bool:
    ctrl = LQR(A, B, Q, R, tau_max=tau_max)
    r = simulate(p, [0.0, theta0, 0.0, 0.0], T_FINAL, DT, controller=ctrl, stop=fallen)
    return r["t"][-1] >= T_FINAL - DT / 2 and abs(r["s"][-1, 1]) < np.radians(1)


angles = np.arange(1, 90)
fig, ax = plt.subplots(figsize=(8, 2.5))
for row, tau_max in enumerate((None, TAU_MAX)):
    ok = np.array([recovers(np.radians(a), tau_max) for a in angles])
    label = "no torque limit" if tau_max is None else f"|tau| <= {tau_max} N m"
    first_fail = angles[~ok][0] if (~ok).any() else None
    print(f"{label}: recovers up to {angles[ok].max() if ok.any() else 0} deg"
          + (f", first failure at {first_fail} deg" if first_fail else ""))
    ax.scatter(angles, np.full_like(angles, row), c=np.where(ok, "tab:green", "tab:red"), s=12)
ax.set_yticks([0, 1], ["no limit", f"|tau| <= {TAU_MAX}"])
ax.set_xlabel(r"initial tilt $\theta_0$ [deg]")
ax.set_title("Region of attraction of the LQR (green = recovers)")
fig.tight_layout()
OUT.mkdir(parents=True, exist_ok=True)
fig.savefig(OUT / "roa.png", dpi=150)
print(f"results saved in {OUT}/")
