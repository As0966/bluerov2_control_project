"""State Feedback Controller — Pure PID Version"""
import numpy as np
from bluerov2_control.trajectory import reference_trajectory


class StateFeedbackController:
    """
    Standard PID State Feedback Controller.
    Follows the real mathematical algorithm: tau = -Kp*e - Kd*edot - Ki*int(e)
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final
        
        # Pure PID Gains - Tuned for smoothness and tracking
        self.Kp = np.diag([25.0, 25.0, 25.0, 10.0])
        self.Kd = np.diag([40.0, 40.0, 40.0, 15.0])
        self.Ki = np.diag([2.0, 2.0, 2.0, 1.0])
        
        self.tau_limits = np.array([35.0, 35.0, 35.0, 10.0])
        self.int_e_eta = np.zeros(4)

    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def compute_control(self, t, X, ref=None):
        dt = 0.01 # Standard control step
        if ref is None:
            ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
        
        eta = X[0:4]
        nu = X[4:8]
        psi = eta[3]
        
        # 1. Calculate Errors in World Frame
        e_eta = eta - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        
        # 2. Update Integral
        self.int_e_eta += e_eta * dt
        self.int_e_eta = np.clip(self.int_e_eta, -5.0, 5.0) # Anti-windup
        
        # 3. Velocity Error (Body Frame)
        # Standard PID state feedback often uses error in body frame for force application
        J = self.model.rotation_matrix(psi)
        e_eta_body = J.T @ e_eta
        int_e_body = J.T @ self.int_e_eta
        
        # Velocity error: nu_actual - J^T * eta_dot_d
        nu_d = J.T @ ref["eta_dot_d"]
        e_nu = nu - nu_d
        
        # 4. PID Control Law: tau = -Kp*e_pos - Kd*e_vel - Ki*e_int + Gravity
        tau = (
            - self.Kp @ e_eta_body 
            - self.Kd @ e_nu 
            - self.Ki @ int_e_body 
            + self.model.g_vector(X)
        )
        
        return np.clip(tau, -self.tau_limits, self.tau_limits)
