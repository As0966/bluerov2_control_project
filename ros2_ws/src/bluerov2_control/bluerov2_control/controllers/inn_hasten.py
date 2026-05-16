import numpy as np
from bluerov2_control.trajectory import reference_trajectory
from bluerov2_control.controllers.backstepping import BacksteppingController

class HastenINNController:
    """
    Adaptive Inverse Neural Network (INN) Controller.
    Trained online via Backpropagation to imitate a Backstepping Teacher.
    Features: [e_eta(4), e_nu(4), nu_d(4), nu(4)] = 16 dimensions
    """

    def __init__(self, model, trajectory_shape="spline", T_final=60.0):
        self.model = model
        self.trajectory_shape = trajectory_shape
        self.T_final = T_final
        
        # Teacher Controller (Expert)
        self.teacher = BacksteppingController(model)
        
        # NN Architecture
        self.input_dim = 16
        self.hidden_dim = 32
        self.output_dim = 4
        
        # High-Gain Input Scaling for faster feature activation
        self.V = np.random.randn(self.input_dim, self.hidden_dim) * 0.5 # Increased from 0.1
        self.b_v = np.random.randn(self.hidden_dim) * 0.1
        
        # Trainable weights
        self.W = np.zeros((self.hidden_dim, self.output_dim))
        
        # Flash Learning Parameters (Decaying high-gain)
        self.eta_base = 2.5     # Very high initial gain
        self.eta_min = 0.5      # Stable long-term gain
        self.decay_const = 0.2  # Decays over ~10 seconds
        
        self.momentum = 0.6     
        self.sigma_mod = 0.0001 
        
        # Internal states for momentum
        self.W_velocity = np.zeros((self.hidden_dim, self.output_dim))
        
        self.tau_limits = np.array([35.0, 35.0, 35.0, 10.0])

    def get_phi(self, features):
        z = np.dot(features, self.V) + self.b_v
        return np.tanh(z)

    def get_features(self, X, ref):
        eta = X[0:4]
        nu = X[4:8]
        eta_d = ref["eta_d"]
        nu_d = self.teacher.desired_body_velocity(X, ref)
        
        e_eta = eta - eta_d
        e_eta[3] = (e_eta[3] + np.pi) % (2.0 * np.pi) - np.pi
        e_nu = nu - nu_d
        
        return np.hstack((e_eta, e_nu, nu_d, nu))

    def compute_control(self, t, X, ref=None):
        if ref is None:
            ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
            
        features = self.get_features(X, ref)
        phi = self.get_phi(features)
        
        # INN Output
        tau_inn = np.dot(phi, self.W)
        
        return np.clip(tau_inn, -self.tau_limits, self.tau_limits)

    def update(self, t, X, dt):
        ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.T_final)
        features = self.get_features(X, ref)
        phi = self.get_phi(features)
        
        # 1. Expert Signal (Teacher)
        tau_teacher = self.teacher.compute_control(t, X, ref=ref)
        
        # 2. Time-Decaying Learning Rate for "Flash Learning"
        # eta(t) = eta_min + (eta_base - eta_min) * exp(-decay * t)
        eta_t = self.eta_min + (self.eta_base - self.eta_min) * np.exp(-self.decay_const * t)
        
        # 3. NN Output
        tau_inn = np.dot(phi, self.W)
        
        # 4. Error for Backpropagation
        error = tau_teacher - tau_inn
        
        # 5. Weight Update with Momentum (Faster & Smoother)
        self.W_velocity = self.momentum * self.W_velocity + eta_t * np.outer(phi, error)
        W_dot = self.W_velocity - self.sigma_mod * self.W
        
        self.W += W_dot * dt
        return self.W
