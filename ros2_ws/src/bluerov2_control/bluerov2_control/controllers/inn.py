import numpy as np
from bluerov2_control.trajectory import reference_trajectory
from bluerov2_control.controllers.backstepping import BacksteppingController

class INNController:
    """
    NEMESIS-Adaptive Inverse Neural Network (INN) Controller.
    
    This implementation uses a Single Hidden Layer (SHL) Neural Network
    with online weight adaptation derived from Lyapunov stability theory.
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final
        
        # Nominal Expert (Backstepping) - Provides the base control law
        self.teacher = BacksteppingController(model)
        
        # NN Architecture Parameters
        # Features: [e_eta(4), e_nu(4), nu_d(4), nu(4)] = 16 dimensions
        # (Removed integral error to avoid internal state mutation in solver)
        self.input_dim = 16  
        self.hidden_dim = 50 
        self.output_dim = 4
        
        # Fixed Hidden Layer Weights (V) - Randomly initialized but constant
        np.random.seed(42)
        self.V = np.random.randn(self.input_dim, self.hidden_dim) * 0.1
        self.b_v = np.random.randn(self.hidden_dim) * 0.05
        
        # Adaptation Gains
        self.Gamma = 15.0 * np.eye(self.hidden_dim) 
        self.sigma = 0.01 # Sigma-modification
        
        # Filtered Error Gain (s = e_nu + Lambda * e_eta)
        self.Lambda = 1.0 * np.eye(4)
        
        # Nominal weights for initialization
        self.W_nominal = np.zeros((self.hidden_dim, self.output_dim))

    def get_features(self, X, ref):
        """Extract the 16-dimensional feature vector."""
        eta = X[0:4]
        nu = X[4:8]
        eta_d = ref["eta_d"]
        
        e_eta = eta - eta_d
        e_eta[3] = (e_eta[3] + np.pi) % (2.0 * np.pi) - np.pi
        
        nu_d = self.teacher.desired_body_velocity(X, ref)
        e_nu = nu - nu_d
        
        return np.hstack((e_eta, e_nu, nu_d, nu))

    def get_phi(self, features):
        """Compute hidden layer activations."""
        z = np.dot(features, self.V) + self.b_v
        return np.tanh(z)

    def compute_control(self, t, X, W, ref=None):
        if ref is None:
            ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
            
        # 1. Nominal Control
        tau_nom = self.teacher.compute_control(t, X, ref=ref)
        
        # 2. NN Compensation
        features = self.get_features(X, ref)
        phi = self.get_phi(features)
        tau_nn = np.dot(phi, W)
        
        tau = tau_nom + tau_nn
        return np.clip(tau, -self.teacher.tau_limits, self.teacher.tau_limits)

    def adaptation_dynamics(self, X, W, ref):
        features = self.get_features(X, ref)
        phi = self.get_phi(features)
        
        eta = X[0:4]
        nu = X[4:8]
        eta_d = ref["eta_d"]
        nu_d = self.teacher.desired_body_velocity(X, ref)
        
        e_eta = eta - eta_d
        e_eta[3] = (e_eta[3] + np.pi) % (2.0 * np.pi) - np.pi
        e_nu = nu - nu_d
        
        # Filtered Error
        s = e_nu + self.Lambda @ e_eta
        
        # Stable Adaptation Law: W_dot = -Gamma * (Phi * s^T + sigma * W)
        # The negative sign ensures that the NN output opposes the tracking error.
        W_dot = -self.Gamma @ (np.outer(phi, s) + self.sigma * W)
        
        return W_dot

    def update(self, t, X, W, dt):
        ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
        W_dot = self.adaptation_dynamics(X, W, ref)
        return W + W_dot * dt


