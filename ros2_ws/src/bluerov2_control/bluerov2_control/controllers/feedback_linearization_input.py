import numpy as np
from bluerov2_control.trajectory import reference_trajectory

class HastenInputFeedbackLinearizationController:
    """
    Velocity-level Feedback Linearization (Input-State) as per HASTEN_AUV.pdf Section 5.1.
    Algorithm: tau = M * (nu_dot_d - K_nu * e_nu) + D * nu
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final
        
        # Increased Velocity Tracking Gain to reduce radius drift
        self.K_nu = np.diag([15.0, 15.0, 15.0, 5.0])
        self.tau_limits = np.array([35.0, 35.0, 35.0, 10.0])

    def compute_control(self, t, X, ref=None):
        if ref is None:
            ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
            
        nu = X[4:8]
        
        # In Input-State FL, we track desired body velocities directly
        # Desired body velocity and acceleration from trajectory
        J = self.model.rotation_matrix(X[3])
        nu_d = J.T @ ref["eta_dot_d"]
        
        # Approximate nu_dot_d (In a pure velocity controller, this is often feedforward)
        # We can use the analytical derivative or a small dt approximation
        nu_dot_d = np.zeros(4) # Simplified for pure velocity tracking
        
        e_nu = nu - nu_d
        
        # Control Law: tau = M * (nu_dot_d - K_nu * e_nu) + D * nu + g(eta)
        tau = self.model.M @ (nu_dot_d - self.K_nu @ e_nu) + self.model.D @ nu + self.model.g_vector(X)
        
        return np.clip(tau, -self.tau_limits, self.tau_limits)
