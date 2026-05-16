import numpy as np
from bluerov2_control.trajectory import reference_trajectory

class HastenIOFeedbackLinearizationController:
    """
    Trajectory-level Feedback Linearization (Input-Output) as per HASTEN_AUV.pdf Section 5.2.
    Algorithm: 
    1. nu_d = J^-1(psi) * (eta_dot_d - K_eta * e_eta)
    2. tau = M * (nu_dot_d - K_nu * e_nu) + D * nu
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final
        
        # Increased Gains to minimize tracking radius error
        self.K_eta = np.diag([5.0, 5.0, 5.0, 2.0])
        self.K_nu = np.diag([15.0, 15.0, 15.0, 5.0])
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
        
        # 1. Outer Loop: Desired Body Velocity
        e_eta = eta - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        
        J = self.model.rotation_matrix(psi)
        nu_d = J.T @ (ref["eta_dot_d"] - self.K_eta @ e_eta)
        
        # 2. Inner Loop: Dynamics Compensation
        e_nu = nu - nu_d
        
        # Simplified acceleration feedforward
        nu_dot_d = np.zeros(4) 
        
        # Dynamics + Gravity Compensation
        tau = self.model.M @ (nu_dot_d - self.K_nu @ e_nu) + self.model.D @ nu + self.model.g_vector(X)
        
        return np.clip(tau, -self.tau_limits, self.tau_limits)
