import numpy as np
from bluerov2_control.trajectory import reference_trajectory

class AdaptiveBacksteppingController:
    """
    Professional Adaptive Integral Backstepping Controller.
    
    Features:
    - Regressor Matrix (Y) for unified parameter estimation.
    - Integral Action in the kinematic loop to eliminate steady-state error.
    - Robust Adaptation with Sigma-Modification to prevent parameter drift.
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final

        # 1. Control Gains
        self.K_eta = np.diag([3.0, 3.0, 3.0, 1.5])    # Proportional
        self.K_i   = np.diag([0.5, 0.5, 0.5, 0.2])    # Integral
        self.K_nu  = np.diag([40.0, 40.0, 40.0, 15.0]) # Velocity tracking
        
        # 2. Adaptation Gains (Gamma)
        # We estimate 8 parameters: [m_u, m_v, m_w, I_z, d_u, d_v, d_w, d_r]
        self.Gamma = 10.0 * np.eye(8)
        self.sigma = 0.05 # Sigma-modification (robustness)
        
        # Nominal parameter vector for initialization
        self.theta_nominal = np.array([
            model.m_u, model.m_v, model.m_w, model.I_z, # Mass
            model.d_u, model.d_v, model.d_w, model.d_r  # Damping
        ])
        
        self.tau_limits = np.array([35.0, 35.0, 35.0, 10.0])

    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def get_regressor(self, nu, nu_d_dot):
        """
        Construct the Regressor Matrix Y such that M*nu_d_dot + D*nu = Y * theta.
        Y is 4x8.
        """
        Y = np.zeros((4, 8))
        # Mass part (M * nu_d_dot)
        Y[0, 0] = nu_d_dot[0]
        Y[1, 1] = nu_d_dot[1]
        Y[2, 2] = nu_d_dot[2]
        Y[3, 3] = nu_d_dot[3]
        # Damping part (D * nu)
        Y[0, 4] = nu[0]
        Y[1, 5] = nu[1]
        Y[2, 6] = nu[2]
        Y[3, 7] = nu[3]
        return Y

    def desired_body_velocity(self, X, ref, z_eta):
        """Kinematic outer-loop with Integral action."""
        eta = X[0:4]
        psi = eta[3]
        J = self.model.rotation_matrix(psi)
        
        eta_d = ref["eta_d"]
        eta_dot_d = ref["eta_dot_d"]
        
        e_eta = eta - eta_d
        e_eta[3] = self.wrap_angle(e_eta[3])
        
        # Virtual control law (nu_d) including Integral term z_eta
        return J.T @ (eta_dot_d - self.K_eta @ e_eta - self.K_i @ z_eta)

    def desired_body_acceleration(self, X, ref, z_eta):
        """Derivative of nu_d (Analytical)."""
        psi = X[3]
        r = X[7]
        eta_d = ref["eta_d"]
        eta_dot_d = ref["eta_dot_d"]
        eta_ddot_d = ref["eta_ddot_d"]
        
        e_eta = X[0:4] - eta_d
        e_eta[3] = self.wrap_angle(e_eta[3])
        
        J = self.model.rotation_matrix(psi)
        J_inv = J.T # Rotation matrix is orthogonal
        
        # J_dot calculation
        c, s = np.cos(psi), np.sin(psi)
        J_dot = np.array([
            [-s*r, -c*r, 0, 0],
            [ c*r, -s*r, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ])
        
        nu = X[4:8]
        e_dot_eta = J @ nu - eta_dot_d
        
        # Derivative of J^T * (eta_dot_d - K_eta * e_eta - K_i * z_eta)
        # Using d/dt(J^T) = -J^T * J_dot * J^T
        term_inner = eta_dot_d - self.K_eta @ e_eta - self.K_i @ z_eta
        term_inner_dot = eta_ddot_d - self.K_eta @ e_dot_eta - self.K_i @ e_eta
        
        nu_d_dot = J_inv @ term_inner_dot - J_inv @ J_dot @ J_inv @ term_inner
        return nu_d_dot

    def compute_control(self, t, X, theta, z_eta, ref=None):
        if ref is None:
            ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
            
        eta = X[0:4]
        nu = X[4:8]
        
        # 1. Virtual Control (Kinematics)
        nu_d = self.desired_body_velocity(X, ref, z_eta)
        
        # 2. Virtual Control Derivative
        nu_d_dot = self.desired_body_acceleration(X, ref, z_eta)
        
        # 3. Error signals
        e_eta = eta - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        e_nu = nu - nu_d
        
        # 4. Control Law (Dynamics)
        # Using Regressor Matrix Y * theta
        Y = self.get_regressor(nu, nu_d_dot)
        J = self.model.rotation_matrix(eta[3])
        
        # tau = Y*theta - K_nu*e_nu - J^T*e_eta + g
        tau = Y @ theta - self.K_nu @ e_nu - J.T @ e_eta + self.model.g_vector(X)
        
        return np.clip(tau, -self.tau_limits, self.tau_limits)

    def adaptation_dynamics(self, X, nu_d, nu_d_dot, theta):
        """
        Lyapunov-based Parameter Update Law.
        dot_theta = -Gamma * Y^T * e_nu - Gamma * sigma * (theta - theta_nominal)
        """
        nu = X[4:8]
        e_nu = nu - nu_d
        Y = self.get_regressor(nu, nu_d_dot)
        
        # Adaptation Law with Sigma-Modification for robustness
        theta_dot = -self.Gamma @ (Y.T @ e_nu + self.sigma * (theta - self.theta_nominal))
        
        return theta_dot

    def update(self, t, X, theta, z_eta, dt):
        """High-level update for integration."""
        ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
        
        nu_d = self.desired_body_velocity(X, ref, z_eta)
        nu_d_dot = self.desired_body_acceleration(X, ref, z_eta)
        
        theta_dot = self.adaptation_dynamics(X, nu_d, nu_d_dot, theta)
        
        # Update integral state
        e_eta = X[0:4] - ref["eta_d"]
        e_eta[3] = self.wrap_angle(e_eta[3])
        z_eta_new = z_eta + e_eta * dt
        z_eta_new = np.clip(z_eta_new, -5.0, 5.0) # Anti-windup
        
        theta_new = theta + theta_dot * dt
        # Physical consistency: parameters (mass, damping) must be positive
        theta_new = np.maximum(theta_new, 0.05)
        
        return theta_new, z_eta_new

