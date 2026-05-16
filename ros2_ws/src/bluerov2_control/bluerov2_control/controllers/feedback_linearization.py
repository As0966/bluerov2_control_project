"""Feedback Linearization Controller — Pure Model Inversion Version"""
import numpy as np
from bluerov2_control.trajectory import reference_trajectory


class FeedbackLinearizationController:
    """
    Standard Feedback Linearization for 4-DOF BlueROV2.
    Algorithm: tau = M*nu_dot_d + C*nu + D*nu + g
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final
        
        # Feedback Gains - Tuned for stability and precision
        self.K_eta = np.diag([5.0, 5.0, 5.0, 2.0])
        self.K_nu = np.diag([30.0, 30.0, 30.0, 10.0])
        self.tau_limits = np.array([35.0, 35.0, 35.0, 10.0])

    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def compute_control(self, t, X):
        eta = X[0:4]
        nu = X[4:8]
        psi = eta[3]
        r = nu[3]
        
        ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
        
        # 1. Kinematic Error
        e_eta = eta - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        
        # 2. Desired Velocities (Kinematic Control Law)
        # eta_dot_ref = eta_dot_d - K_eta * e_eta
        eta_dot_ref = ref["eta_dot_d"] - self.K_eta @ e_eta
        
        J = self.model.rotation_matrix(psi)
        nu_d = J.T @ eta_dot_ref
        
        # 3. Acceleration Feedforward (Simplified to Core Algorithm)
        # In pure FL, we often assume nu_dot_d is the derivative of nu_d
        # To keep it "real" and avoid "unreal" J_dot if not requested, 
        # we use the standard model-based acceleration.
        dt = 0.01
        ref_next = reference_trajectory(t + dt, shape=self.trajectory_shape, T_final=self.T_final)
        eta_dot_ref_next = ref_next["eta_dot_d"] - self.K_eta @ ( (eta + J@nu*dt) - ref_next["eta_d"] )
        nu_d_next = self.model.rotation_matrix(psi + r*dt).T @ eta_dot_ref_next
        nu_dot_d = (nu_d_next - nu_d) / dt
        
        # 4. Control Law: tau = M * nu_dot_d + D * nu + g - K_nu * (nu - nu_d)
        e_nu = nu - nu_d
        tau = (
            self.model.M @ nu_dot_d 
            + self.model.D @ nu 
            + self.model.g_vector(X) 
            - self.K_nu @ e_nu
        )
        
        return np.clip(tau, -self.tau_limits, self.tau_limits)
