"""Experiment 06 - state estimation with simulated sensors.

1. Open loop: the LQR uses the TRUE state, while the estimators run alongside on
   the sensor readings. Compares theta from: accelerometer only, gyro integration
   only, complementary filter.                                  -> theta_estimates.png
2. Closed loop: the LQR uses the ESTIMATED state (realistic).   -> closed_loop.png, lqr_estimated.gif

Choose ALPHA below (and write ComplementaryFilter.update in balancebot/estimators.py).
Run from the repository root:  python experiments/06_estimation.py
"""
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from balancebot import RobotParams, viz
from balancebot.controllers import LQR, VoltageCommand, WithEstimator
from balancebot.dynamics import forward_dynamics
from balancebot.estimators import ComplementaryFilter, StateEstimator, accel_angle
from balancebot.linearize import linearize
from balancebot.motor import MotorParams, torque
from balancebot.sensors import Sensors, SensorParams
from balancebot.simulate import simulate

OUT = Path("runs/06_estimation")
OUT.mkdir(parents=True, exist_ok=True)
p, m, sp = RobotParams(), MotorParams(), SensorParams()
A, B = linearize(p)
DT, T_FINAL, THETA0 = 5e-3, 8.0, np.radians(10.0)
ALPHA = 0.998                    # complementary filter weight: T = 2.5 s (see the alpha sweep in the chat)
Q, R = np.diag([1.0, 10.0, 1.0, 1.0]), np.array([[100.0]])
fallen = lambda t, s: abs(s[1]) >= np.pi / 2
print(f"filter time constant T = {1e3 * ALPHA * DT / (1 - ALPHA):.0f} ms")

# ---------------------------------------------------------------- 1. open loop: compare estimates
ctrl = VoltageCommand(LQR(A, B, Q, R), p, m)
res = simulate(p, [0.0, THETA0, 0.0, 0.0], T_FINAL, DT, controller=ctrl, stop=fallen, motor=m)
# the robot is held still at the start: the initial estimate comes from the accelerometer
sens, cf = Sensors(p, sp), ComplementaryFilter(ALPHA, DT, theta0=THETA0)
th_acc, th_gyro, th_cf = [], [], []
g_int = THETA0                    # same initial knowledge as the filter
for k, s in enumerate(res["s"]):
    s_dot = forward_dynamics(s, res["tau"][k], p)
    gyro, af, au = sens.imu(s, s_dot)
    th_acc.append(accel_angle(af, au))
    th_gyro.append(g_int)
    g_int += gyro * DT
    th_cf.append(cf.update(gyro, af, au)[0])
t = res["t"]
fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
for lab, th in (("accelerometer only", th_acc), ("gyro integration only", th_gyro),
                ("complementary filter", th_cf)):
    ax[0].plot(t, np.degrees(th), lw=0.9, label=lab)
    ax[1].plot(t, np.degrees(np.array(th) - res["s"][:, 1]), lw=0.9, label=lab)
    print(f"{lab:22s}: rms error = {np.degrees(np.sqrt(np.mean((np.array(th) - res['s'][:, 1])**2))):.2f} deg")
ax[0].plot(t, np.degrees(res["s"][:, 1]), "k", lw=1.5, label="true")
ax[0].set_ylabel(r"$\theta$ [deg]"); ax[1].set_ylabel("error [deg]"); ax[1].set_xlabel("t [s]")
ax[1].set_ylim(-5, 5)
for a in ax:
    a.grid(alpha=0.3); a.legend(fontsize=8)
fig.tight_layout(); fig.savefig(OUT / "theta_estimates.png", dpi=130)

# ---------------------------------------------------------------- 2. closed loop on the estimate
est = StateEstimator(p, sp, DT, ALPHA, theta0=THETA0)
ctrl = WithEstimator(VoltageCommand(LQR(A, B, Q, R), p, m), Sensors(p, sp), est, p, motor=m)
res2 = simulate(p, [0.0, THETA0, 0.0, 0.0], T_FINAL, DT, controller=ctrl, stop=fallen, motor=m)
s_hat = np.array(ctrl.log)
n = min(len(s_hat), len(res2["t"]))
fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
labels = [r"$x$ [cm]", r"$\theta$ [deg]", r"$\dot x$ [m/s]", r"$\dot\theta$ [deg/s]"]
scale = [100, np.degrees(1), 1, np.degrees(1)]
for i, a in enumerate(ax):
    a.plot(res2["t"][:n], res2["s"][:n, i] * scale[i], "k", label="true")
    a.plot(res2["t"][:n], s_hat[:n, i] * scale[i], lw=0.8, label="estimated")
    a.set_ylabel(labels[i]); a.grid(alpha=0.3)
ax[0].legend(); ax[-1].set_xlabel("t [s]")
fig.tight_layout(); fig.savefig(OUT / "closed_loop.png", dpi=130)
if res2["t"][-1] < T_FINAL - DT / 2:
    print(f"closed loop on the estimate: FELL after {res2['t'][-1]:.2f} s")
else:
    print(f"closed loop on the estimate: final theta = {np.degrees(res2['s'][-1, 1]):.2f} deg, "
          f"final x = {100 * res2['s'][-1, 0]:.1f} cm")
viz.animate(res2, p, path=OUT / "lqr_estimated.gif")
print(f"results saved in {OUT}/")
