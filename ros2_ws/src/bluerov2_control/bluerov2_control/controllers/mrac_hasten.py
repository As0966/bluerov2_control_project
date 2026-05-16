import numpy as np
from bluerov2_control.trajectory import reference_trajectory

class HastenMRACController:
    """
    Standard MRAC Controller as per HASTEN_AUV.pdf Section 6.
    Reference Model: nu_m_dot = a_m * nu_m + b_m * nu_d
    Control Law: tau = theta1 * nu + theta2 * nu_d
    Adaptation: theta1_dot = -gamma1 * e * nu, theta2_dot = -gamma2 * e * nu_d
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final
        
        # Reference Model Parameters (a_m < 0 for stability)
        self.a_m = -2.0 * np.ones(4)
        self.b_m = 2.0 * np.ones(4)
        
        # Increased Adaptation Gains for faster tracking convergence
        self.gamma1 = 20.0 * np.ones(4)
        self.gamma2 = 20.0 * np.ones(4)
        
        # Nominal theta for initialization (from model)
        m_diag = np.diag(self.model.M)
        d_diag = np.diag(self.model.D)
        # Assuming nominal parameters match the reference model initially
        self.theta1_nominal = d_diag + m_diag * self.a_m
        self.theta2_nominal = m_diag * self.b_m
        
        self.tau_limits = np.array([35.0, 35.0, 35.0, 10.0])

    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def desired_body_velocity(self, X, ref):
        eta = X[0:4]
        e_eta = eta - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        J = self.model.rotation_matrix(eta[3])
        # Outer loop gain for nu_d calculation
        K_eta = 2.0 * np.eye(4)
        return J.T @ (ref["eta_dot_d"] - K_eta @ e_eta)

    def reference_model_dynamics(self, nu_m, nu_d):
        return self.a_m * nu_m + self.b_m * nu_d

    def adaptation_dynamics(self, nu, nu_d, nu_m, theta1, theta2):
        e = nu - nu_m
        theta1_dot = -self.gamma1 * e * nu
        theta2_dot = -self.gamma2 * e * nu_d
        return theta1_dot, theta2_dot

    def compute_control(self, X, nu, nu_d, theta1, theta2):
        tau = theta1 * nu + theta2 * nu_d + self.model.g_vector(X)
        return np.clip(tau, -self.tau_limits, self.tau_limits)

    def update(self, t, X, nu_m, theta1, theta2, dt):
        ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
        nu = X[4:8]
        nu_d = self.desired_body_velocity(X, ref)
        
        nu_m_dot = self.reference_model_dynamics(nu_m, nu_d)
        t1_dot, t2_dot = self.adaptation_dynamics(nu, nu_d, nu_m, theta1, theta2)
        
        return nu_m + dt*nu_m_dot, theta1 + dt*t1_dot, theta2 + dt*t2_dot
