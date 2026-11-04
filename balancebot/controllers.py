"""Balance controllers.

Every controller is a callable  controller(t, s) -> tau  (same interface as
simulate()), where s = [x, theta, x_dot, theta_dot] and tau is the total wheel
torque [N m]. Sign convention: tau > 0 accelerates the robot towards +x and
tilts the body backwards (theta_dd < 0), see B in balancebot.linearize.

TODO marks the parts to be written.
"""
from typing import Optional
import numpy as np
import scipy


def saturate(tau: float, tau_max: Optional[float]) -> float:
    """Clip tau to [-tau_max, tau_max] (no limit if tau_max is None)."""
    if tau_max is None:
        return float(tau)
    return float(np.clip(tau, -tau_max, tau_max))


# ======================================================================== PID
class PID:
    """PID on the tilt angle: error e = theta_ref - theta.

        tau = kp * e + ki * integral(e dt) + kd * de/dt

    Notes for the implementation:
      * the derivative of the error is -theta_dot when theta_ref is constant:
        use the measured theta_dot (s[3]) instead of differentiating theta
        numerically (on the robot it comes directly from the gyroscope);
      * the integral is accumulated once per call: the simulator calls the
        controller exactly once per step of length dt;
      * think about the overall sign: with theta > 0 the robot falls towards +x;
        which sign of tau brings it back? (look at the 4th entry of B).
    """

    def __init__(self, kp: float, ki: float, kd: float, dt: float,
                 theta_ref: float = 0.0, tau_max: Optional[float] = None):
        """
        Args:
            kp:        proportional gain [N m / rad].
            ki:        integral gain [N m / (rad s)].
            kd:        derivative gain [N m s / rad].
            dt:        controller period [s]: the time between two calls, equal to
                       the simulation step (used to accumulate the integral).
            theta_ref: desired tilt [rad], 0 = upright.
            tau_max:   torque limit [N m], output clipped to [-tau_max, tau_max];
                       None = no limit.
        """
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.theta_ref = theta_ref
        self.tau_max = tau_max
        self.reset()

    def reset(self) -> None:
        """Clear the internal state (integral of the error)."""
        self.integral = 0.0

    def __call__(self, t: float, s: np.ndarray) -> float:
        """Compute the torque for the current state. Called once per step.

        Args:
            t: current time [s]
            s: current state [x, theta, x_dot, theta_dot];

        Returns:
            tau: total wheel torque [N m], after saturation.
        """
        current_theta = s[1]
        current_theta_dot = s[3]
        err = current_theta - self.theta_ref
        self.integral += err * self.dt

        tau = (self.kp * err) + (self.ki * self.integral) + (self.kd * current_theta_dot)
        return saturate(tau, self.tau_max)


# ======================================================================== LQR
def lqr_gain(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Optimal state-feedback gain K (1x4) of the continuous-time LQR problem.

    Minimises  J = integral( s^T Q s + tau^T R tau ) dt  for  s_dot = A s + B tau.

    Args:
        A: state matrix (4x4) of the linearised model.
        B: input matrix (4x1) of the linearised model.
        Q: state weights (4x4, symmetric, positive semi-definite); Q[i, i] is the
           cost of a deviation of state i, in the order [x, theta, x_dot, theta_dot].
        R: input weight (1x1, positive); cost of using torque.

    Returns:
        K: gain (1x4) such that tau = -K s.
    """
    # find P (4x4, symmetric) solving the continuous algebraic Riccati equation
    #            A^T P + P A - P B R^-1 B^T P + Q = 0
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    
    # K = R^-1 B^T P
    K = scipy.linalg.inv(R) @ B.T @ P

    return K


class LQR:
    """State feedback  tau = -K (s - s_ref)."""

    def __init__(self, A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray,
                 s_ref=None, tau_max: Optional[float] = None):
        """
        Args:
            A, B:    linearised model (4x4 and 4x1), from balancebot.linearize.
            Q, R:    LQR weights (4x4 and 1x1), see lqr_gain.
            s_ref:   desired state [x, theta, x_dot, theta_dot]; None = all zeros
                     (upright, at rest, at x = 0).
            tau_max: torque limit [N m]; None = no limit.
        """
        self.K = lqr_gain(A, B, Q, R)
        self.s_ref = np.zeros(4) if s_ref is None else np.asarray(s_ref, dtype=float)
        self.tau_max = tau_max

    def __call__(self, t: float, s: np.ndarray) -> float:
        """Compute the torque for the current state. Called once per step.

        Args:
            t: current time [s] (not needed by the LQR, kept for the common
               controller interface used by simulate()).
            s: current state [x, theta, x_dot, theta_dot] (shape (4,)).

        Returns:
            tau: total wheel torque [N m] as a float, after saturation.
        """
        error = s - self.s_ref
        tau = -float((self.K @ error).item())
        
        if self.tau_max is not None:
            tau = float(np.clip(tau, -self.tau_max, self.tau_max))

        return tau
