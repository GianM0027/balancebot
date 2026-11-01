"""Generate the simulation figures and numbers used in the report.

Run from the repository root:  python report/figures/make_figures.py
Outputs (in report/figures/): free_fall_states.pdf, free_fall_energy.pdf,
free_fall_frames.pdf, results.txt
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches, transforms

from balancebot import RobotParams
from balancebot.integrators import euler_step, rk4_step
from balancebot.simulate import simulate

OUT = Path(__file__).parent
plt.rcParams.update({"font.family": "serif", "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "lines.linewidth": 1.3})

p = RobotParams()
s0 = [0.0, np.radians(3.0), 0.0, 0.0]
dt = 5e-3
fallen = lambda t, s: abs(s[1]) >= np.pi / 2
res = {k: simulate(p, s0, 3.0, dt, integrator=k, stop=fallen) for k in ("euler", "rk4")}
r = res["rk4"]
lines = []

# ---------------------------------------------------------------- states
fig, ax = plt.subplots(2, 2, figsize=(6.3, 3.6), sharex=True)
labels = [r"$x$ [cm]", r"$\theta$ [deg]", r"$\dot x$ [m/s]", r"$\dot\theta$ [deg/s]"]
scale = [100, np.degrees(1), 1, np.degrees(1)]
for i, a in enumerate(ax.T.flat):
    a.plot(r["t"], r["s"][:, i] * scale[i], color="k")
    a.set_ylabel(labels[i])
for a in ax[1]:
    a.set_xlabel(r"$t$ [s]")
fig.tight_layout()
fig.savefig(OUT / "free_fall_states.pdf")

# ---------------------------------------------------------------- energy
fig, ax = plt.subplots(figsize=(6.3, 2.4))
for name, style in (("euler", "--"), ("rk4", "-")):
    rr = res[name]
    E = rr["T"] + rr["V"]
    drift = (E - E[0]) / abs(E[0])
    ax.plot(rr["t"], drift, style, color="k", label={"euler": "explicit Euler", "rk4": "RK4"}[name])
    lines.append(f"{name}: max relative energy drift = {np.max(np.abs(drift)):.2e}")
ax.set_xlabel(r"$t$ [s]")
ax.set_ylabel(r"$(E - E_0)/E_0$")
ax.legend()
fig.tight_layout()
fig.savefig(OUT / "free_fall_energy.pdf")

# ---------------------------------------------------------------- snapshots
h, w = 2.6 * p.l, 0.8 * p.l
times = [0.0, 0.15, 0.25, 0.30, r["t"][-1]]
idx = [int(np.argmin(np.abs(r["t"] - tt))) for tt in times]
fig, axs = plt.subplots(1, len(idx), figsize=(6.3, 1.9), sharey=True)
xmin, xmax = r["s"][:, 0].min() - 0.05, r["s"][:, 0].max() + h + 0.02
for a, k in zip(axs, idx):
    x, th = r["s"][k, 0], r["s"][k, 1]
    body = patches.Rectangle((-w / 2, -0.3 * p.r), w, h, fc="0.85", ec="k", lw=0.8)
    body.set_transform(transforms.Affine2D().rotate(-th).translate(x, 0) + a.transData)
    a.add_patch(body)
    a.add_patch(patches.Circle((x, 0), p.r, fc="white", ec="k", lw=0.8))
    a.plot([x, x - p.r * np.sin(x / p.r)], [0, -p.r * np.cos(x / p.r)], "k-", lw=0.6)
    a.axhline(-p.r, color="k", lw=0.8)
    a.axvline(0, color="0.6", lw=0.5, ls=":")
    a.set_aspect("equal")
    a.set_xlim(xmin, xmax)
    a.set_ylim(-p.r - 0.01, h + 0.03)
    a.set_title(f"t = {r['t'][k]:.2f} s\n" + r"$\theta$" + f" = {np.degrees(th):.0f}" + r"$^\circ$", fontsize=8)
    a.grid(False)
    a.set_yticks([])
    a.tick_params(labelsize=7)
fig.supxlabel(r"$x$ [m]", fontsize=9)
fig.tight_layout()
fig.savefig(OUT / "free_fall_frames.pdf")

# ---------------------------------------------------------------- integrator order (test problem)
OMEGA, T_END = 2.0, 2.0
f = lambda t, s: np.array([s[1], -OMEGA**2 * s[0]])
def err(step, h_):
    s, t = np.array([1.0, 0.0]), 0.0
    for _ in range(int(round(T_END / h_))):
        s = step(f, t, s, h_); t += h_
    return abs(s[0] - np.cos(OMEGA * T_END))
for name, step, h1 in (("euler", euler_step, 1e-3), ("rk4", rk4_step, 2e-2)):
    e1, e2 = err(step, h1), err(step, h1 / 2)
    lines.append(f"{name}: error dt={h1:g} {e1:.3e}, dt={h1/2:g} {e2:.3e}, ratio {e1/e2:.2f}")

a_coef = p.m_w + p.I_w / p.r**2 + p.m_b
p_x = a_coef * r["s"][:, 2] + p.m_b * p.l * np.cos(r["s"][:, 1]) * r["s"][:, 3]
lines.append(f"max |p_x| (dL/dx_dot) along RK4 trajectory = {np.max(np.abs(p_x)):.2e} kg m/s")
lines.insert(0, f"fall time (|theta| = 90 deg): {r['t'][-1]:.3f} s")
lines.insert(1, f"wheel displacement at the fall: {100 * r['s'][-1, 0]:.2f} cm")
lines.insert(2, "theta at snapshots: " +
             ", ".join(f"t={r['t'][k]:.2f}: {np.degrees(r['s'][k, 1]):.1f} deg" for k in idx))
(OUT / "results.txt").write_text("\n".join(lines) + "\n")
print("\n".join(lines))
