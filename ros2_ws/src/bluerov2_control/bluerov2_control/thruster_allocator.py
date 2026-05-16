"""
Thruster Allocation Matrix (TAM) for the BlueROV2 Heavy 6-thruster configuration.

Maps 4-DOF generalized forces tau = [Fx, Fy, Fz, Mz] to 6 individual thruster forces.

Thruster layout (from actuators.xacro):
  T0: front-right  horizontal, angle  45° (0.7854 rad)  at (0.1355, -0.1)
  T1: front-left   horizontal, angle -45° (-0.7854 rad) at (0.1355,  0.1)
  T2: rear-right   horizontal, angle 135° (2.3562 rad)  at (-0.1475, -0.1)
  T3: rear-left    horizontal, angle -135° (-2.3562 rad) at (-0.1475, 0.1)
  T4: left vertical   at (0.0025, -0.1105)
  T5: right vertical  at (0.0025,  0.1105)
"""
import numpy as np


class ThrusterAllocator:
    """
    Converts 4-DOF body-frame forces to 6 individual thruster commands.
    """

    # Max force per thruster (Newtons) — T200 thruster approx limit
    MAX_THRUST = 40.0

    def __init__(self):
        # Thruster configuration
        angles = [0.7854, -0.7854, 2.3562, -2.3562]  # rad
        positions = [
            (0.1355, -0.1),
            (0.1355,  0.1),
            (-0.1475, -0.1),
            (-0.1475,  0.1),
        ]

        # Build the 4×6 configuration matrix B
        # tau = B @ f  →  f = B_pinv @ tau
        B = np.zeros((4, 6))

        for i in range(4):
            ca = np.cos(angles[i])
            sa = np.sin(angles[i])
            xi, yi = positions[i]

            B[0, i] = ca          # Fx contribution
            B[1, i] = sa          # Fy contribution
            B[3, i] = xi * sa - yi * ca  # Mz (torque arm)

        # Vertical thrusters: with pitch=-90° pose and axis=[1,0,0],
        # thrust direction in body frame is [0,0,-1] (downward).
        # Positive command → body -Z force, so coefficient is -1.
        B[2, 4] = -1.0
        B[2, 5] = -1.0

        self.B = B
        self.B_pinv = np.linalg.pinv(B)

    def allocate(self, tau: np.ndarray) -> np.ndarray:
        """
        Convert 4-DOF tau = [Fx, Fy, Fz, Mz] to 6 thruster forces.

        Args:
            tau: (4,) array of generalized forces

        Returns:
            f: (6,) array of individual thruster forces (Newtons)
        """
        f = self.B_pinv @ tau
        f = np.clip(f, -self.MAX_THRUST, self.MAX_THRUST)
        return f
