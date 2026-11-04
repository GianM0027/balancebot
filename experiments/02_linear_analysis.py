"""Linear analysis of the robot around the upright equilibrium.

1. A and B_lin of the linearised model  s_dot = A s + B_lin tau,  s = [x, theta, x_dot, theta_dot]
2. check against a numerical linearisation of the nonlinear forward_dynamics
3. eigenvalues of A and comparison of the fall rate with the nonlinear simulation
4. controllability: rank of [B, AB, A^2 B, A^3 B]
"""
import numpy as np

from balancebot import RobotParams
from balancebot.dynamics import forward_dynamics
from balancebot.linearize import controllability_matrix, linearize, linearize_numerical
from balancebot.simulate import simulate

p = RobotParams()

# ---------------------------------------------------------------- 1. analytic A, B_lin
A, B_lin = linearize(p)

np.set_printoptions(precision=4, suppress=True)
print("A =\n", A)
print("B_lin =\n", B_lin)

# ---------------------------------------------------------------- 2. numerical linearisation (central differences)
A_num, B_num = linearize_numerical(lambda s, tau: forward_dynamics(s, tau, p), np.zeros(4))
print(f"\nmax |A - A_num| = {np.max(np.abs(A - A_num)):.1e}")
print(f"max |B - B_num| = {np.max(np.abs(B_lin - B_num)):.1e}")

# ---------------------------------------------------------------- 3. eigenvalues and fall rate
eig = np.linalg.eigvals(A)
lam = eig.real.max()
print("\neigenvalues of A:", np.sort_complex(eig))
print(f"unstable eigenvalue lambda = {lam:.2f} 1/s  (time constant 1/lambda = {1000 / lam:.0f} ms)")

theta0 = np.radians(3.0)
res = simulate(p, [0.0, theta0, 0.0, 0.0], 0.35, 1e-3, integrator="rk4")
print("\n  t [s]   linear theta0*cosh(lambda t) [deg]   nonlinear simulation [deg]")
for t in (0.05, 0.10, 0.15, 0.20, 0.25, 0.30):
    k = int(np.argmin(np.abs(res["t"] - t)))
    print(f"  {t:.2f}   {np.degrees(theta0 * np.cosh(lam * t)):10.1f}"
          f"                     {np.degrees(res['s'][k, 1]):10.1f}")

# ---------------------------------------------------------------- 4. controllability
C = controllability_matrix(A, B_lin)
print(f"\nrank of controllability matrix = {np.linalg.matrix_rank(C)} (of 4)")
