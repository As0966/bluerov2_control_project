"""Nonlinear Observer-Based Controller — Standard Version"""
import numpy as np
from bluerov2_control.trajectory import reference_trajectory


class NonlinearObserverBasedController:
    """
    Robust Nonlinear Observer-Based Controller.
    Algorithm: tau = M*nu_dot_d + D*nu - K_nu*e_nu - J^T*e_eta + g
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final
        
        # Control Gains (Consistent with Backstepping Expert)
        self.K_eta = np.diag([3.5, 3.5, 3.5, 1.5])
        self.K_nu = np.diag([50.0, 50.0, 50.0, 20.0])
        
        # Observer Gains (Reduced for stability)
        self.L = np.zeros((8, 4))
        self.L[0:4, 0:4] = 4.0 * np.eye(4)
        self.L[4:8, 0:4] = 8.0 * np.eye(4)
        
        # Bias estimation gain
        self.Ki = 0.5 * np.eye(4)
        self.bias_damping = 0.01
        
        self.tau_limits = np.array([35.0, 35.0, 35.0, 10.0])

    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def desired_body_velocity(self, Xhat, ref):
        eta_hat = Xhat[0:4]
        psi = eta_hat[3]
        e_eta = eta_hat - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        J = self.model.rotation_matrix(psi)
        return J.T @ (ref["eta_dot_d"] - self.K_eta @ e_eta)

    def desired_body_acceleration(self, t, Xhat, ref):
        psi = Xhat[3]
        r = Xhat[7]
        eta_d = ref["eta_d"]
        eta_dot_d = ref["eta_dot_d"]
        eta_ddot_d = ref["eta_ddot_d"]
        
        e_eta = Xhat[0:4] - eta_d
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
        nu_hat = Xhat[4:8]
        e_dot_eta = J @ nu_hat - eta_dot_d
        term2 = J_inv @ (eta_ddot_d - self.K_eta @ e_dot_eta)
        return term1 + term2

    def compute_control(self, t, Xhat, ref=None):
        if ref is None:
            ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
        
        nu_hat = Xhat[4:8]
        eta_hat = Xhat[0:4]
        e_eta = eta_hat - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        
        nu_d = self.desired_body_velocity(Xhat, ref)
        nu_dot_d = self.desired_body_acceleration(t, Xhat, ref)
        e_nu = nu_hat - nu_d
        
        J = self.model.rotation_matrix(Xhat[3])
        
        tau = (
            self.model.M @ nu_dot_d
            + self.model.D @ nu_hat 
            - self.K_nu @ e_nu 
            - J.T @ e_eta 
            + self.model.g_vector(Xhat)
        )
        return np.clip(tau, -self.tau_limits, self.tau_limits)

    def observer_dynamics(self, t, Xhat, y_meas, tau, bias_hat=None):
        innovation = y_meas - Xhat[0:4]
        innovation[3] = self.wrap_angle(innovation[3])
        
        if bias_hat is None:
            bias_hat = np.zeros(4)
            
        nu = Xhat[4:8]
        J = self.model.rotation_matrix(Xhat[3])
        eta_dot = J @ nu
        # bias_hat represents external disturbance in body frame
        nu_dot = self.model.M_inv @ (tau - self.model.D @ nu - self.model.g_vector(Xhat) - bias_hat)
        
        X_dot_model = np.concatenate([eta_dot, nu_dot])
        
        # We must apply gains in the correct frames:
        # L1 (position) acts in inertial frame.
        # L2 (velocity) acts in body frame, so we rotate the innovation.
        L1 = self.L[0:4, 0:4]
        L2 = self.L[4:8, 0:4]
        
        innovation_body = J.T @ innovation
        
        Xhat_dot = np.concatenate([
            eta_dot + L1 @ innovation,
            nu_dot + L2 @ innovation_body
        ])
        
        # Bias adaptation (integral of innovation in body frame)
        bias_dot = -self.Ki @ innovation_body - self.bias_damping * bias_hat
        
        return Xhat_dot, bias_dot

    def observer_update(self, Xhat, y_meas, tau, dt, bias_hat=None):
        if bias_hat is None:
            bias_hat = np.zeros(4)
        Xhat_dot, bias_dot = self.observer_dynamics(0, Xhat, y_meas, tau, bias_hat=bias_hat)
        return Xhat + dt * Xhat_dot, bias_hat + dt * bias_dot
