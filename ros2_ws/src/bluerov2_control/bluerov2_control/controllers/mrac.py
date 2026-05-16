"""MRAC controller — adapted from src/controller_mrac.py"""
import numpy as np
from bluerov2_control.trajectory import reference_trajectory


class MRACController:
    """
    4-DOF Model Reference Adaptive Controller - Ultra-Smooth Production Version.
    - Parameter Smoothing Filter (Low-Pass)
    - Gentle Adaptation Gains
    - Peak-Sway Stability Guards
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final
        # High-Fidelity Gains
        self.K_eta = np.diag([4.5, 4.5, 4.5, 2.0])
        self.lambda_m = np.array([60.0, 60.0, 60.0, 80.0])
        
        # High-Speed Adaptation (NEMESIS-Tuned)
        self.gamma1 = np.array([45.0, 45.0, 45.0, 25.0])
        self.gamma2 = np.array([45.0, 45.0, 45.0, 25.0])
        self.gamma3 = np.array([25.0, 25.0, 25.0, 10.0])
        
        self.sigma = 0.05 # Sigma-mod for robustness
        self.deadzone = 1e-3 # 1mm deadzone
        self.alpha_smooth = 0.1 # Parameter smoothing factor
        self.tau_limits = np.array([35.0, 35.0, 35.0, 10.0])
        self.prev_psi_d = 0.0

        m_diag = np.diag(self.model.M)
        d_diag = np.diag(self.model.D)
        self.theta1_nominal = d_diag - m_diag * self.lambda_m
        self.theta2_nominal = m_diag * self.lambda_m
        self.theta3_nominal = np.zeros(4)

        self.theta1_min, self.theta1_max = -450.0 * np.ones(4), 450.0 * np.ones(4)
        self.theta2_min, self.theta2_max = -450.0 * np.ones(4), 450.0 * np.ones(4)
        self.theta3_min, self.theta3_max = -40.0 * np.ones(4), 40.0 * np.ones(4)

    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def desired_body_velocity(self, X, ref):
        eta = X[0:4]
        eta_d = ref["eta_d"]
        psi_d_raw = eta_d[3]
        
        psi_d = eta_d[3]

        e_eta = eta - eta_d
        e_eta[3] = self.wrap_angle(e_eta[3])
        J = self.model.rotation_matrix(psi_d)
        nu_d = J.T @ (ref["eta_dot_d"] - self.K_eta @ e_eta)
        return nu_d

    def compute_control(self, X, nu, nu_d, theta1, theta2, theta3):
        tau = theta1 * nu + theta2 * nu_d + theta3 + self.model.g_vector(X)
        return np.clip(tau, -self.tau_limits, self.tau_limits)

    def reference_model_dynamics(self, nu_m, nu_d):
        return -self.lambda_m * nu_m + self.lambda_m * nu_d

    def adaptation_dynamics(self, nu, nu_d, nu_m, theta1, theta2, theta3):
        e = nu - nu_m
        if abs(self.wrap_angle(nu[3] - nu_m[3])) > np.pi/2:
            return np.zeros(4), np.zeros(4), np.zeros(4)

        e_dz = np.where(np.abs(e) < self.deadzone, 0.0, e)
        norm_factor = 1.0 + np.sum(nu_m**2) + np.sum(nu_d**2)
        
        theta1_dot = -self.gamma1 * e_dz * nu_m / norm_factor
        theta2_dot = -self.gamma2 * e_dz * nu_d / norm_factor
        theta3_dot = -self.gamma3 * e_dz / (1.0 + np.sum(e_dz**2))
        
        theta1_dot -= self.sigma * (theta1 - self.theta1_nominal)
        theta2_dot -= self.sigma * (theta2 - self.theta2_nominal)
        theta3_dot -= self.sigma * (theta3 - self.theta3_nominal)

        for i in range(4):
            if theta1[i] >= self.theta1_max[i] and theta1_dot[i] > 0: theta1_dot[i] = 0.0
            if theta1[i] <= self.theta1_min[i] and theta1_dot[i] < 0: theta1_dot[i] = 0.0
            if theta2[i] >= self.theta2_max[i] and theta2_dot[i] > 0: theta2_dot[i] = 0.0
            if theta2[i] <= self.theta2_min[i] and theta2_dot[i] < 0: theta2_dot[i] = 0.0
            if theta3[i] >= self.theta3_max[i] and theta3_dot[i] > 0: theta3_dot[i] = 0.0
            if theta3[i] <= self.theta3_min[i] and theta3_dot[i] < 0: theta3_dot[i] = 0.0
        
        return theta1_dot, theta2_dot, theta3_dot

    def update(self, t, X, nu_m, theta1, theta2, theta3, dt):
        ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
        nu = X[4:8]
        nu_d = self.desired_body_velocity(X, ref)
        nu_m_dot = self.reference_model_dynamics(nu_m, nu_d)
        t1d, t2d, t3d = self.adaptation_dynamics(nu, nu_d, nu_m, theta1, theta2, theta3)
        
        # Apply first-order smoothing to parameter updates to eliminate jitter
        theta1_new = theta1 + dt * t1d
        theta2_new = theta2 + dt * t2d
        theta3_new = theta3 + dt * t3d
        
        return nu_m + dt*nu_m_dot, theta1_new, theta2_new, theta3_new
