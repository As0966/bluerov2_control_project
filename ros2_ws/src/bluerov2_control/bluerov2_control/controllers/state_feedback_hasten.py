import numpy as np
from bluerov2_control.trajectory import reference_trajectory

class HastenStateFeedbackController:
    """
    Pure Linear State Feedback Controller as specified in HASTEN_AUV.pdf Section 3.
    Algorithm: tau = -K * (X - X_d)
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final
        
        # Increased Gains to reduce steady-state error in curved paths (radius drift)
        self.K_eta = np.diag([25.0, 25.0, 25.0, 10.0])
        self.K_nu = np.diag([40.0, 40.0, 40.0, 15.0])
        
        self.tau_limits = np.array([35.0, 35.0, 35.0, 10.0])

    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def compute_control(self, t, X, ref=None):
        if ref is None:
            ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
        
        eta = X[0:4]
        nu = X[4:8]
        psi = eta[3]
        
        # 1. Calculate Errors
        e_eta = eta - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        
        # Desired body velocity for tracking
        J = self.model.rotation_matrix(psi)
        nu_d = J.T @ ref["eta_dot_d"]
        e_nu = nu - nu_d
        
        # 2. Linear Control Law: tau = -K_eta*e_eta_body - K_nu*e_nu + Gravity
        # We rotate position error to body frame to apply forces correctly
        e_eta_body = J.T @ e_eta
        
        tau = - self.K_eta @ e_eta_body - self.K_nu @ e_nu + self.model.g_vector(X)
        
        return np.clip(tau, -self.tau_limits, self.tau_limits)
