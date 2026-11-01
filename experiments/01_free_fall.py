"""Experiment 01 - free fall with the motor off.

The robot starts at rest, tilted by 3 degrees, with tau = 0. Outputs, in
runs/01_free_fall/:
  states.png      x, theta and velocities while the robot falls (RK4)
  energy.png      total-energy drift, explicit Euler vs RK4, same step
  free_fall.gif   side-view animation, 4x slower than real time

The simulation stops when the body reaches the horizontal (|theta| = 90 deg):
the model has no ground contact for the body, so beyond that it is not physical.

Run from the repository root:  python experiments/01_free_fall.py
"""
from pathlib import Path

import numpy as np

from balancebot import RobotParams
from balancebot.simulate import simulate
from balancebot import viz

OUT = Path("runs/01_free_fall")
p = RobotParams()
s0 = [0.0, np.radians(3.0), 0.0, 0.0]
dt = 5e-3
fallen = lambda t, s: abs(s[1]) >= np.pi / 2

res = {name: simulate(p, s0, t_final=3.0, dt=dt, integrator=name, stop=fallen)
       for name in ("euler", "rk4")}

r = res["rk4"]
print(f"fall time (|theta| = 90 deg): {r['t'][-1]:.3f} s")
print(f"wheel displacement at the fall: {100 * r['s'][-1, 0]:.1f} cm")
for name, rr in res.items():
    E = rr["T"] + rr["V"]
    print(f"{name:5s}: max relative energy drift = {np.max(np.abs(E - E[0])) / abs(E[0]):.2e}")

viz.plot_states(r, title="Free fall, motor off (RK4)", path=OUT / "states.png")
viz.plot_energy({f"Euler, dt = {dt*1e3:g} ms": res["euler"], f"RK4, dt = {dt*1e3:g} ms": r},
                path=OUT / "energy.png")
viz.animate(r, p, path=OUT / "free_fall.gif", speed=0.25)
print(f"results saved in {OUT}/")
