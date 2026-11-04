"""Plots and 2-D animation of simulation results."""
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation, patches, transforms

from .params import RobotParams


def plot_states(res: dict, title: str = "", path: Optional[Path] = None):
    """Plot x, theta and their velocities against time."""
    t, s = res["t"], res["s"]
    fig, ax = plt.subplots(2, 2, figsize=(10, 6), sharex=True)
    labels = [r"$x$ [m]", r"$\theta$ [deg]", r"$\dot x$ [m/s]", r"$\dot\theta$ [deg/s]"]
    scale = [1.0, np.degrees(1.0), 1.0, np.degrees(1.0)]
    for i, a in enumerate(ax.T.flat):
        a.plot(t, s[:, i] * scale[i])
        a.set_ylabel(labels[i])
        a.grid(alpha=0.3)
    for a in ax[1]:
        a.set_xlabel("t [s]")
    fig.suptitle(title)
    fig.tight_layout()
    _save(fig, path)
    return fig


def plot_energy(results: dict, path: Optional[Path] = None):
    """Relative drift of the total energy, (E(t) - E(0)) / E(0), for several runs.

    Args:
        results: {label: simulation result}.
    """
    fig, ax = plt.subplots(figsize=(8, 4))
    for label, res in results.items():
        E = res["T"] + res["V"]
        ax.plot(res["t"], (E - E[0]) / abs(E[0]), label=label)
    ax.set_xlabel("t [s]")
    ax.set_ylabel(r"$(E - E_0)/E_0$")
    ax.set_title("Total energy drift (motor off, no friction)")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    _save(fig, path)
    return fig


def animate(res: dict, p: RobotParams, path: Optional[Path] = None, fps: int = 30, speed: float = 1.0,
            body_height: Optional[float] = None, body_width: Optional[float] = None):
    """2-D side-view animation. Saved as .gif or .mp4 (needs ffmpeg) if path is given.

    speed: playback speed (0.25 = four times slower than real time).
    """
    t, s = res["t"], res["s"]
    h = body_height or 2.6 * p.l
    w = body_width or 1.2 * h / 3
    every = max(1, int(round(speed / (fps * (t[1] - t[0])))))
    frames = list(range(0, len(t), every)) + [len(t) - 1] * fps  # hold the last frame 1 s

    fig, ax = plt.subplots(figsize=(7, 4))
    x_span = (s[:, 0].min() - 2 * h, s[:, 0].max() + 2 * h)
    ax.set_xlim(*x_span)
    ax.set_ylim(-p.r - 0.02, p.r + h * 1.3)
    ax.set_aspect("equal")
    ax.axhline(-p.r, color="k", lw=1)
    ax.set_xlabel("x [m]")

    body = patches.Rectangle((-w / 2, -0.3 * p.r), w, h, fc="0.9", ec="k")
    wheel = patches.Circle((0, 0), p.r, fc="white", ec="k", zorder=3)
    spoke, = ax.plot([], [], "k-", lw=1, zorder=4)
    com, = ax.plot([], [], "ko", ms=4, zorder=5)
    ax.add_patch(body)
    ax.add_patch(wheel)
    label = ax.text(0.02, 0.92, "", transform=ax.transAxes)

    def update(k):
        x, th = s[k, 0], s[k, 1]
        phi = x / p.r
        body.set_transform(transforms.Affine2D().rotate(-th).translate(x, 0) + ax.transData)
        wheel.center = (x, 0)
        spoke.set_data([x, x - p.r * np.sin(phi)], [0, -p.r * np.cos(phi)])  # clockwise rotation by phi
        com.set_data([x + p.l * np.sin(th)], [p.l * np.cos(th)])
        label.set_text(f"t = {t[k]:.3f} s   theta = {np.degrees(th):5.1f} deg   (x{speed:g})")
        return body, wheel, spoke, com, label

    anim = animation.FuncAnimation(fig, update, frames=frames, interval=1000 / fps, blit=False)
    if path is not None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        writer = animation.PillowWriter(fps=fps) if path.suffix == ".gif" else animation.FFMpegWriter(fps=fps)
        anim.save(path, writer=writer)
    return anim


def _save(fig, path):
    if path is not None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150)


def plot_control(res: dict, title: str = "", path: Optional[Path] = None,
                 tau_max: Optional[float] = None):
    """Plot theta, x and the applied torque against time."""
    t, s, tau = res["t"], res["s"], res["tau"]
    fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    ax[0].plot(t, np.degrees(s[:, 1]))
    ax[0].set_ylabel(r"$\theta$ [deg]")
    ax[1].plot(t, 100 * s[:, 0])
    ax[1].set_ylabel(r"$x$ [cm]")
    ax[2].plot(t, tau)
    if tau_max is not None:
        for v in (tau_max, -tau_max):
            ax[2].axhline(v, color="k", ls="--", lw=0.8)
    ax[2].set_ylabel(r"$\tau$ [N m]")
    ax[2].set_xlabel("t [s]")
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle(title)
    fig.tight_layout()
    _save(fig, path)
    return fig
