"""
BlueROV2 4-DOF dynamic model.
Includes mass, damping, and restoring forces (gravity/buoyancy).
"""
import numpy as np

class BlueROV2Model4DOF:
    def __init__(self):
        # Mass + Added Mass
        self.m_u = 11.5
        self.m_v = 13.5
        self.m_w = 14.0
        self.I_z = 2.8

        # Damping (Linear approximation)
        self.d_u = 6.0
        self.d_v = 7.0
        self.d_w = 8.0
        self.d_r = 1.5

        # Restoring Forces (NED: positive Z is down)
        self.mass = 10.0
        self.g = 9.81
        self.W = self.mass * self.g
        self.B = self.W * 1.01  # 1% positively buoyant
        
        # Center of buoyancy and gravity relative to body origin
        self.z_g = 0.02  # 2cm below origin
        self.z_b = 0.0   # at origin

        self.M = np.diag([self.m_u, self.m_v, self.m_w, self.I_z])
        self.D = np.diag([self.d_u, self.d_v, self.d_w, self.d_r])
        self.M_inv = np.linalg.inv(self.M)

    def rotation_matrix(self, psi: float) -> np.ndarray:
        c = np.cos(psi)
        s = np.sin(psi)
        return np.array([
            [c, -s, 0.0, 0.0],
            [s,  c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

    def g_vector(self, X: np.ndarray) -> np.ndarray:
        # f_z = -(W-B)
        f_z = -(self.W - self.B)
        return np.array([0.0, 0.0, f_z, 0.0])

    def disturbance(self, t, enabled=False):
        """Standardized time-varying external disturbances (currents/waves)."""
        if not enabled:
            return np.zeros(4)
        return np.array([
            1.5 * np.sin(0.2 * t) + 0.5,
            1.2 * np.cos(0.15 * t) - 0.3,
            0.8 * np.sin(0.1 * t),
            0.4 * np.cos(0.25 * t)
        ])

    def dynamics(self, t, X, tau, disturbance_enabled=False):
        nu = X[4:8]
        psi = X[3]
        J = self.rotation_matrix(psi)
        eta_dot = J @ nu
        
        d = self.disturbance(t, enabled=disturbance_enabled)
        nu_dot = self.M_inv @ (tau - self.D @ nu - self.g_vector(X) - d)
        return np.concatenate([eta_dot, nu_dot])
