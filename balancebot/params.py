"""Physical parameters of the robot.

The values below are NOMINAL ESTIMATES used until the real parts are measured.
Replace them with measured values (Phase 3) and keep a note of how each one
was obtained.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class RobotParams:
    # --- body (frame, electronics, battery, motor housings) ---
    m_b: float = 0.55      # [kg]     estimate: frame 150 g + 2 motors 190 g + battery 120 g + electronics/screws 90 g
    I_b: float = 1.7e-3    # [kg m^2] estimate: box 0.18 m x 0.06 m, m (h^2 + d^2) / 12
    l: float = 0.07        # [m]      estimate: axle -> body CoM

    # --- wheels (both together) ---
    m_w: float = 0.06      # [kg]     estimate: 2 x 30 g
    r: float = 0.0325      # [m]      65 mm wheels
    I_w: float = 5.0e-5    # [kg m^2] estimate: ~0.75 m_w r^2 (between disc and ring)
    # NOTE: the inertia of the motor rotor, seen through the gearbox, adds
    # N^2 * J_rotor to I_w and may well dominate it. To be identified.

    # --- environment ---
    g: float = 9.81        # [m/s^2]
    