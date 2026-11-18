"""Physical parameters of the robot.

The values below are NOMINAL ESTIMATES used until the real parts are measured.
Replace them with measured values (Phase 3) and keep a note of how each one
was obtained.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class RobotParams:
    # --- body (frame, electronics, battery, motor housings) ---
    m_b: float = 0.69      # [kg]     estimate: frame 160 g + 2 motors 190 g + battery 250 g + electronics/screws 90 g
    I_b: float = 3.8e-3    # [kg m^2] estimate: point masses (battery on top at 0.17 m, motors on the axle)
                           #          + frame as a 0.20 m rod, about the body CoM
    l: float = 0.095       # [m]      estimate: axle -> body CoM, battery mounted on top

    # --- wheels (both together) ---
    m_w: float = 0.08      # [kg]     estimate: 2 x (printed rim 30 g + O-ring 10 g)
    r: float = 0.046       # [m]      O-ring 90x80x5 mm stretched on a ~82.5 mm groove
    I_w: float = 1.3e-4    # [kg m^2] estimate: ~0.75 m_w r^2 (between disc and ring)
    # NOTE: the inertia of the motor rotor, seen through the gearbox, adds
    # N^2 * J_rotor to I_w and may well dominate it. To be identified.

    # --- environment ---
    g: float = 9.81        # [m/s^2]
