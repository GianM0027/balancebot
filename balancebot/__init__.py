"""Simulation and control of a two-wheeled self-balancing robot.

State convention used everywhere (SI units, signs as in the report):

    s = [x, theta, x_dot, theta_dot]

    x      horizontal position of the wheel axle               [m]
    theta  pitch of the body from the vertical, + towards +x   [rad]
"""
from .params import RobotParams

__all__ = ["RobotParams"]
