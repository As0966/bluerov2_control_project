"""Backstepping controller — Robust Version (No Integral)"""
import numpy as np
from bluerov2_control.trajectory import reference_trajectory


class BacksteppingController:
    """
    Robust 4-DOF Backstepping Controller.
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final
        # Robust Research Gains (High Stiffness, no Integral)
        self.K_eta = np.diag([3.5, 3.5, 3.5, 1.5])
        self.K_nu = np.diag([50.0, 50.0, 50.0, 20.0])
        self.K_s = np.diag([3.0, 3.0, 3.0, 1.5])
        self.phi = np.array([0.15, 0.15, 0.15, 0.1])
        self.tau_limits = np.array([35.0, 35.0, 35.0, 10.0])
        
        self.int_e_eta = np.zeros(4) # Not used in pure robust BS

    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    @staticmethod
    def sat(x):
        return np.clip(x, -1.0, 1.0)

    def desired_body_velocity(self, X, ref):
        eta = X[0:4]
        psi = eta[3]
        e_eta = eta - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        J = self.model.rotation_matrix(psi)
        return J.T @ (ref["eta_dot_d"] - self.K_eta @ e_eta)

    def desired_body_acceleration(self, t, X, ref):
        psi = X[3]
        r = X[7]
        eta_d = ref["eta_d"]
        eta_dot_d = ref["eta_dot_d"]
        eta_ddot_d = ref["eta_ddot_d"]
        
        e_eta = X[0:4] - eta_d
        e_eta[3] = self.wrap_angle(e_eta[3])
        
        J = self.model.rotation_matrix(psi)
        J_inv = J.T
        
        c = np.cos(psi)
        s = np.sin(psi)
        J_dot = np.array([
            [-s*r, -c*r, 0, 0],
            [ c*r, -s*r, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ])
        
        term1 = -J_inv @ J_dot @ J_inv @ (eta_dot_d - self.K_eta @ e_eta)
        nu = X[4:8]
        e_dot_eta = J @ nu - eta_dot_d
        term2 = J_inv @ (eta_ddot_d - self.K_eta @ e_dot_eta)
        return term1 + term2

    def compute_control(self, t, X, ref=None):
        nu = X[4:8]
        if ref is None:
            ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
        
        eta = X[0:4]
        e_eta = eta - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        
        nu_d = self.desired_body_velocity(X, ref)
        nu_dot_d = self.desired_body_acceleration(t, X, ref)
        e_nu = nu - nu_d
        robust_term = self.K_s @ self.sat(e_nu / self.phi)
        
        J = self.model.rotation_matrix(X[3])
        
        tau = (
            self.model.M @ nu_dot_d
            + self.model.D @ nu 
            - self.K_nu @ e_nu 
            - J.T @ e_eta 
            + self.model.g_vector(X)
            - robust_term
        )
        return np.clip(tau, -self.tau_limits, self.tau_limits)
