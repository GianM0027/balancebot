"""Generate the figures and numbers used in the report.

Run from the repository root, one part at a time (each takes from seconds to ~2 minutes):

    python report/figures/make_figures.py free_fall
    python report/figures/make_figures.py linear
    python report/figures/make_figures.py control
    python report/figures/make_figures.py motor
    python report/figures/make_figures.py estimation
    python report/figures/make_figures.py sweep

Each part writes its PDFs to report/figures/ and its numbers to report/figures/results_<part>.txt.
"""
import sys
from dataclasses import replace
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches, transforms

from balancebot import RobotParams
from balancebot.controllers import LQR, PID, VoltageCommand, WithEstimator
from balancebot.dynamics import forward_dynamics
from balancebot.estimators import ComplementaryFilter, StateEstimator, accel_angle
from balancebot.integrators import euler_step, rk4_step
from balancebot.linearize import controllability_matrix, linearize, linearize_numerical
from balancebot.motor import MotorParams, torque_limits
from balancebot.sensors import Sensors, SensorParams
from balancebot.simulate import simulate

OUT = Path(__file__).parent
plt.rcParams.update({"font.family": "serif", "font.size": 9, "axes.grid": True,
                     "grid.alpha": 0.3, "lines.linewidth": 1.2})
p, m, sp = RobotParams(), MotorParams(), SensorParams()
A, B = linearize(p)
DT = 5e-3
Q, R = np.diag([1.0, 10.0, 1.0, 1.0]), np.array([[100.0]])
KP, KI, KD = 0.61, 0.026, 0.01
fallen = lambda t, s: abs(s[1]) >= np.pi / 2


def write(part: str, lines: list) -> None:
    """Print the numbers of a part and save them to results_<part>.txt.

    Args:
        part:  name of the part.
        lines: lines of text to save.
    """
    (OUT / f"results_{part}.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


def recovers(ctrl, theta0: float, motor=None, t_final: float = 5.0) -> bool:
    """True if the controller brings the robot back upright from rest at theta0.

    Args:
        ctrl:    controller (fresh instance).
        theta0:  initial tilt [rad].
        motor:   motor parameters if ctrl returns a voltage, else None.
        t_final: simulated time [s].
    """
    r = simulate(p, [0.0, theta0, 0.0, 0.0], t_final, DT, controller=ctrl, stop=fallen, motor=motor)
    return r["t"][-1] >= t_final - DT / 2 and abs(r["s"][-1, 1]) < np.radians(1)


# ======================================================================== free fall
def free_fall():
    """Free fall with the motor off: snapshots, states, energy, integrator order."""
    s0 = [0.0, np.radians(3.0), 0.0, 0.0]
    res = {k: simulate(p, s0, 3.0, DT, integrator=k, stop=fallen) for k in ("euler", "rk4")}
    r = res["rk4"]
    lines = []
    fig, ax = plt.subplots(2, 2, figsize=(6.3, 3.6), sharex=True)
    labels = [r"$x$ [cm]", r"$\theta$ [deg]", r"$\dot x$ [m/s]", r"$\dot\theta$ [deg/s]"]
    scale = [100, np.degrees(1), 1, np.degrees(1)]
    for i, a in enumerate(ax.T.flat):
        a.plot(r["t"], r["s"][:, i] * scale[i], color="k")
        a.set_ylabel(labels[i])
    for a in ax[1]:
        a.set_xlabel(r"$t$ [s]")
    fig.tight_layout(); fig.savefig(OUT / "free_fall_states.pdf")

    fig, ax = plt.subplots(figsize=(6.3, 2.4))
    for name, style in (("euler", "--"), ("rk4", "-")):
        rr = res[name]
        E = rr["T"] + rr["V"]
        drift = (E - E[0]) / abs(E[0])
        ax.plot(rr["t"], drift, style, color="k", label={"euler": "explicit Euler", "rk4": "RK4"}[name])
        lines.append(f"{name}: max relative energy drift = {np.max(np.abs(drift)):.2e}")
    ax.set_xlabel(r"$t$ [s]"); ax.set_ylabel(r"$(E - E_0)/E_0$"); ax.legend()
    fig.tight_layout(); fig.savefig(OUT / "free_fall_energy.pdf")

    h, w = 2.6 * p.l, 0.8 * p.l
    times = [0.0, 0.15, 0.25, 0.30, r["t"][-1]]
    idx = [int(np.argmin(np.abs(r["t"] - tt))) for tt in times]
    fig, axs = plt.subplots(1, len(idx), figsize=(6.3, 1.9), sharey=True)
    xmin, xmax = r["s"][:, 0].min() - 0.06, r["s"][:, 0].max() + h + 0.02
    for a, k in zip(axs, idx):
        x, th = r["s"][k, 0], r["s"][k, 1]
        body = patches.Rectangle((-w / 2, -0.3 * p.r), w, h, fc="0.85", ec="k", lw=0.8)
        body.set_transform(transforms.Affine2D().rotate(-th).translate(x, 0) + a.transData)
        a.add_patch(body)
        a.add_patch(patches.Circle((x, 0), p.r, fc="white", ec="k", lw=0.8))
        a.plot([x, x - p.r * np.sin(x / p.r)], [0, -p.r * np.cos(x / p.r)], "k-", lw=0.6)
        a.axhline(-p.r, color="k", lw=0.8); a.axvline(0, color="0.6", lw=0.5, ls=":")
        a.set_aspect("equal"); a.set_xlim(xmin, xmax); a.set_ylim(-p.r - 0.01, h + 0.03)
        a.set_title(f"t = {r['t'][k]:.2f} s\n" + r"$\theta$" + f" = {np.degrees(th):.0f}" + r"$^\circ$", fontsize=8)
        a.grid(False); a.set_yticks([]); a.tick_params(labelsize=7)
    fig.supxlabel(r"$x$ [m]", fontsize=9)
    fig.tight_layout(); fig.savefig(OUT / "free_fall_frames.pdf")

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
    lines.append(f"max |p_x| along RK4 trajectory = {np.max(np.abs(p_x)):.2e} kg m/s")
    lines.insert(0, f"fall time (|theta| = 90 deg): {r['t'][-1]:.3f} s")
    lines.insert(1, f"wheel displacement at the fall: {100 * r['s'][-1, 0]:.2f} cm")
    lines.insert(2, "theta at snapshots: " + ", ".join(
        f"t={r['t'][k]:.2f}: {np.degrees(r['s'][k, 1]):.1f} deg" for k in idx))
    write("free_fall", lines)


# ======================================================================== linearisation
def linear():
    """Matrices, eigenvalues, comparison linear vs nonlinear fall."""
    a = p.m_w + p.I_w / p.r**2 + p.m_b
    b0, c = p.m_b * p.l, p.m_b * p.l**2 + p.I_b
    D = a * c - b0**2
    A_num, B_num = linearize_numerical(lambda s, tau: forward_dynamics(s, tau, p), np.zeros(4))
    lam = np.sqrt(A[3, 1])
    lines = [f"a = {a:.4f}, b0 = {b0:.5f}, c = {c:.5f}, D = {D:.4e}",
             f"alpha = {A[2, 1]:.3f}, beta = {A[3, 1]:.3f}, b3 = {B[2, 0]:.3f}, b4 = {B[3, 0]:.3f}",
             f"max |A - A_num| = {np.abs(A - A_num).max():.1e}, max |B - B_num| = {np.abs(B - B_num).max():.1e}",
             f"lambda = {lam:.3f} 1/s, 1/lambda = {1e3 / lam:.1f} ms",
             f"rank C = {np.linalg.matrix_rank(controllability_matrix(A, B))}"]
    th0 = np.radians(3.0)
    r = simulate(p, [0.0, th0, 0.0, 0.0], 0.40, 1e-3, stop=fallen)
    for t in (0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35):
        k = int(np.argmin(np.abs(r["t"] - t)))
        if abs(r["t"][k] - t) < 1e-3:
            lines.append(f"t={t:.2f}: linear {np.degrees(th0 * np.cosh(lam * t)):.1f} deg, "
                         f"nonlinear {np.degrees(r['s'][k, 1]):.1f} deg")
    write("linear", lines)


# ======================================================================== control
def control():
    """PID and LQR with ideal torque: gains, eigenvalues, response from 10 deg."""
    a = p.m_w + p.I_w / p.r**2 + p.m_b
    b0 = p.m_b * p.l
    D = a * (p.m_b * p.l**2 + p.I_b) - b0**2
    mgl = p.m_b * p.g * p.l
    g4 = (a + b0 / p.r) / D                          # |b4|
    kp_min = a * mgl / (a + b0 / p.r)
    w = np.sqrt(g4 * KP - a * mgl / D)
    zeta = g4 * KD / (2 * w)
    kd_crit = 2 * w / g4
    lqr = LQR(A, B, Q, R)
    eig = np.linalg.eigvals(A - B @ lqr.K)
    lines = [f"PID: kp_min = {kp_min:.4f} N m/rad, omega_n = {w:.2f} rad/s, zeta = {zeta:.3f}, "
             f"kd for zeta=1: {kd_crit:.4f}",
             "LQR K = " + ", ".join(f"{k:.4f}" for k in lqr.K.ravel()),
             "LQR closed-loop eigenvalues = " + ", ".join(f"{e:.2f}" for e in eig),
             f"max |lambda| dt = {np.abs(eig).max() * DT:.3f}"]
    th0 = np.radians(10.0)
    fig, ax = plt.subplots(2, 1, figsize=(6.3, 3.6), sharex=True)
    for name, ctrl, style in (("PID", PID(KP, KI, KD, DT), "--"), ("LQR", LQR(A, B, Q, R), "-")):
        r = simulate(p, [0.0, th0, 0.0, 0.0], 8.0, DT, controller=ctrl, stop=fallen)
        ax[0].plot(r["t"], np.degrees(r["s"][:, 1]), style, color="k", label=name)
        ax[1].plot(r["t"], 100 * r["s"][:, 0], style, color="k", label=name)
        lines.append(f"{name} from 10 deg: max |tau| = {np.abs(r['tau']).max():.3f} N m, "
                     f"x(5 s) = {100 * r['s'][int(5 / DT), 0]:.1f} cm, x(8 s) = {100 * r['s'][-1, 0]:.1f} cm, "
                     f"x_dot(8 s) = {100 * r['s'][-1, 2]:.2f} cm/s, max x = {100 * r['s'][:, 0].max():.1f} cm")
    ax[0].set_ylabel(r"$\theta$ [deg]"); ax[1].set_ylabel(r"$x$ [cm]"); ax[1].set_xlabel(r"$t$ [s]")
    ax[0].legend()
    fig.tight_layout(); fig.savefig(OUT / "control_response.pdf")
    write("control", lines)


# ======================================================================== motor
def motor():
    """Motor constants, response with the motor model, region of attraction vs battery."""
    K_t = m.tau_stall / m.i_stall
    K_e = m.V_nom / m.omega_0
    R_w = m.V_nom / m.i_stall
    lines = [f"tau_stall = {m.tau_stall:.4f} N m, omega_0 = {m.omega_0:.3f} rad/s",
             f"R = {R_w:.2f} Ohm, K_t = {K_t:.4f} N m/A, K_e = {K_e:.4f} V s/rad, K_t/K_e = {K_t / K_e:.3f}",
             f"v_max = {m.omega_0 * p.r:.3f} m/s"]
    th0 = np.radians(10.0)
    for name in ("PID", "LQR"):
        tc = PID(KP, KI, KD, DT) if name == "PID" else LQR(A, B, Q, R)
        r = simulate(p, [0.0, th0, 0.0, 0.0], 5.0, DT, controller=VoltageCommand(tc, p, m), stop=fallen, motor=m)
        lim = np.array([torque_limits(s, p, m) for s in r["s"]])
        margin = np.minimum(lim[:, 1] - r["tau"], r["tau"] - lim[:, 0]).min()
        lines.append(f"{name} from 10 deg with motors: max |V| = {np.abs(r['u']).max():.2f} V, "
                     f"max |i| = {np.abs(r['i']).max():.2f} A per motor, min torque margin = {margin:.3f} N m")
    for name in ("PID", "LQR"):
        row = []
        for v in (10.8, 11.1, 12.0, 12.6):
            mv = replace(m, V_batt=v); best = 0
            for deg in range(2, 62, 2):
                tc = PID(KP, KI, KD, DT) if name == "PID" else LQR(A, B, Q, R)
                if recovers(VoltageCommand(tc, p, mv), np.radians(deg), motor=mv):
                    best = deg
                else:
                    break
            row.append(f"{v} V: {best} deg")
        lines.append(f"{name} region of attraction: " + ", ".join(row))
    write("motor", lines)


# ======================================================================== estimation
def estimation():
    """Tilt estimates (accelerometer, gyro, complementary filter) and closed loop on the estimate."""
    alpha, th0, T = 0.998, np.radians(10.0), 8.0
    r = simulate(p, [0.0, th0, 0.0, 0.0], T, DT, controller=VoltageCommand(LQR(A, B, Q, R), p, m),
                 stop=fallen, motor=m)
    sens, cf = Sensors(p, sp), ComplementaryFilter(alpha, DT, theta0=th0)
    est = {"accelerometer": [], "gyroscope": [], "complementary filter": []}
    g_int = th0
    for k, s in enumerate(r["s"]):
        gyro, af, au = sens.imu(s, forward_dynamics(s, r["tau"][k], p))
        est["accelerometer"].append(accel_angle(af, au))
        est["gyroscope"].append(g_int); g_int += gyro * DT
        est["complementary filter"].append(cf.update(gyro, af, au)[0])
    lines = [f"alpha = {alpha}, T = {alpha * DT / (1 - alpha):.3f} s"]
    fig, ax = plt.subplots(figsize=(6.3, 2.6))
    for (name, th), style in zip(est.items(), (":", "--", "-")):
        e = np.degrees(np.array(th) - r["s"][:, 1])
        ax.plot(r["t"], e, style, color="k", lw=0.9, label=name)
        lines.append(f"{name}: rms error {np.sqrt(np.mean(e**2)):.3f} deg, max {np.abs(e).max():.3f} deg")
    ax.set_xlabel(r"$t$ [s]"); ax.set_ylabel(r"$\hat\theta - \theta$ [deg]"); ax.set_ylim(-4, 4); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(OUT / "estimation_error.pdf")
    c = WithEstimator(VoltageCommand(LQR(A, B, Q, R), p, m), Sensors(p, sp),
                      StateEstimator(p, sp, DT, alpha, theta0=th0), p, motor=m)
    r2 = simulate(p, [0.0, th0, 0.0, 0.0], T, DT, controller=c, stop=fallen, motor=m)
    lines.append(f"closed loop on the estimate: t_end = {r2['t'][-1]:.2f} s, theta(8 s) = "
                 f"{np.degrees(r2['s'][-1, 1]):.3f} deg, x(8 s) = {100 * r2['s'][-1, 0]:.2f} cm")
    write("estimation", lines)


# ======================================================================== alpha sweep
def sweep():
    """Closed loop on the estimate for several alpha, 3 noise sequences each."""
    th0, T = np.radians(10.0), 10.0
    lines = ["alpha, T [s], falls, theta error rms [deg], theta error max [deg], max |x| in the last 2 s [cm]"]
    for alpha in (0.95, 0.98, 0.99, 0.995, 0.998, 0.999):
        res, falls = [], 0
        for seed in (0, 1, 2):
            spx = replace(sp, seed=seed)
            c = WithEstimator(VoltageCommand(LQR(A, B, Q, R), p, m), Sensors(p, spx),
                              StateEstimator(p, spx, DT, alpha, theta0=th0), p, motor=m)
            r = simulate(p, [0.0, th0, 0.0, 0.0], T, DT, controller=c, stop=fallen, motor=m)
            if r["t"][-1] < T - DT / 2:
                falls += 1
                continue
            sh = np.array(c.log); n = min(len(sh), len(r["t"]))
            e = np.degrees(sh[:n, 1] - r["s"][:n, 1])
            res.append((np.sqrt(np.mean(e**2)), np.abs(e).max(), 100 * np.abs(r["s"][-400:, 0]).max()))
        mean = np.mean(res, axis=0) if res else [np.nan] * 3
        lines.append(f"{alpha}, {alpha * DT / (1 - alpha):.2f}, {falls}/3, {mean[0]:.2f}, {mean[1]:.2f}, {mean[2]:.1f}")
    write("sweep", lines)


if __name__ == "__main__":
    for part in sys.argv[1:]:
        {"free_fall": free_fall, "linear": linear, "control": control, "motor": motor,
         "estimation": estimation, "sweep": sweep}[part]()
