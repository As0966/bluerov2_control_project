import numpy as np
from bluerov2_control.trajectory import reference_trajectory

class HastenAdaptiveBacksteppingController:
    """
    Pure Adaptive Backstepping Controller.
    Regressor Matrix Y is used for mass and damping estimation.
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final
        
        # Gains
        self.K_eta = np.diag([3.0, 3.0, 3.0, 1.5])
        self.K_nu = np.diag([40.0, 40.0, 40.0, 15.0])
        self.Gamma = 10.0 * np.eye(8) # Adaptation gain
        
        self.theta_nominal = np.array([
            model.m_u, model.m_v, model.m_w, model.I_z, # Mass
            model.d_u, model.d_v, model.d_w, model.d_r  # Damping
        ])
        
        self.tau_limits = np.array([35.0, 35.0, 35.0, 10.0])

    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def get_regressor(self, nu, nu_d_dot):
        Y = np.zeros((4, 8))
        # Mass part
        Y[0, 0] = nu_d_dot[0]; Y[1, 1] = nu_d_dot[1]; Y[2, 2] = nu_d_dot[2]; Y[3, 3] = nu_d_dot[3]
        # Damping part
        Y[0, 4] = nu[0]; Y[1, 5] = nu[1]; Y[2, 6] = nu[2]; Y[3, 7] = nu[3]
        return Y

    def desired_body_velocity(self, X, ref):
        eta = X[0:4]
        e_eta = eta - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        J = self.model.rotation_matrix(eta[3])
        return J.T @ (ref["eta_dot_d"] - self.K_eta @ e_eta)

    def desired_body_acceleration(self, X, ref):
        # Simplified analytical derivative or numerical approximation
        return np.zeros(4) 

    def compute_control(self, t, X, theta, ref=None):
        if ref is None:
            ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
            
        nu = X[4:8]
        nu_d = self.desired_body_velocity(X, ref)
        nu_d_dot = self.desired_body_acceleration(X, ref)
        
        e_eta = X[0:4] - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        e_nu = nu - nu_d
        
        Y = self.get_regressor(nu, nu_d_dot)
        J = self.model.rotation_matrix(X[3])
        
        # tau = Y*theta - K_nu*e_nu - J^T*e_eta + g
        tau = Y @ theta - self.K_nu @ e_nu - J.T @ e_eta + self.model.g_vector(X)
        
        return np.clip(tau, -self.tau_limits, self.tau_limits)

    def adaptation_dynamics(self, X, nu_d, nu_d_dot, theta):
        e_nu = X[4:8] - nu_d
        Y = self.get_regressor(X[4:8], nu_d_dot)
        theta_dot = -self.Gamma @ (Y.T @ e_nu)
        return theta_dot

    def update(self, t, X, theta, dt):
        ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
        nu_d = self.desired_body_velocity(X, ref)
        nu_d_dot = self.desired_body_acceleration(X, ref)
        
        theta_dot = self.adaptation_dynamics(X, nu_d, nu_d_dot, theta)
        theta_new = theta + theta_dot * dt
        return np.maximum(theta_new, 0.05) # Keep physical parameters positive
