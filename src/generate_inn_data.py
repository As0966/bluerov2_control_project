import os
import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp

from bluerov2_control.model import BlueROV2Model4DOF
from bluerov2_control.trajectory import reference_trajectory
from bluerov2_control.controllers.backstepping import BacksteppingController


def build_features_and_target(t, X, teacher, shape, int_e):
    eta = X[0:4]
    nu = X[4:8]
    ref = reference_trajectory(t, shape=shape)
    eta_d = ref["eta_d"]
    
    nu_d = teacher.desired_body_velocity(X, ref)
    e_eta = eta - eta_d
    e_eta[3] = (e_eta[3] + np.pi) % (2.0 * np.pi) - np.pi
    e_nu = nu - nu_d

    # Teacher output (using the integral error we tracked during rollout)
    # Manually update teacher's internal integral state for this calculation
    teacher.int_e_eta = int_e
    tau_bs = teacher.compute_control(t, X, ref)

    # Features: [e_eta (4), e_nu (4), nu_d (4), nu (4), int_e (4)] = 20 dimensions
    features = np.hstack((e_eta, e_nu, nu_d, nu, int_e))
    return np.hstack((features, tau_bs))


def generate_rollout_data(model, teacher, shape):
    t0 = 0.0
    tf = 120.0 
    dt = 0.01 # Matching 100Hz
    t_eval = np.arange(t0, tf + dt, dt)

    ref0 = reference_trajectory(0.0, shape=shape)
    X0 = np.hstack((ref0["eta_d"], np.zeros(4)))
    
    # We must track the integral during the rollout to save it
    teacher.int_e_eta = np.zeros(4)
    
    def dynamics(t, X):
        ref = reference_trajectory(t, shape=shape)
        # Controller internal update (including integral error)
        tau = teacher.compute_control(t, X, ref)
        return model.dynamics(t, X, tau, disturbance_enabled=True)

    sol = solve_ivp(dynamics, (t0, tf), X0, t_eval=t_eval, method="RK45")
    
    rows = []
    current_int_e = np.zeros(4)
    for ti, Xi in zip(sol.t, sol.y.T):
        ref_i = reference_trajectory(ti, shape=shape)
        e_i = Xi[0:4] - ref_i["eta_d"]
        e_i[3] = (e_i[3] + np.pi) % (2.0 * np.pi) - np.pi
        
        # Approximate the integral that the controller saw
        current_int_e += e_i * 0.01
        current_int_e = np.clip(current_int_e, -5.0, 5.0)
        
        rows.append(build_features_and_target(ti, Xi, teacher, shape, current_int_e.copy()))
        
    return rows


def generate_random_perturbation_data(model, teacher, n_samples=60000, seed=42):
    rng = np.random.default_rng(seed)
    rows = []
    shapes = ['spline', 'circle', 'figure8']

    for _ in range(n_samples):
        t = rng.uniform(0.0, 120.0)
        shape = rng.choice(shapes)
        ref = reference_trajectory(t, shape=shape)
        
        e_eta = rng.uniform(-0.5, 0.5, size=4)
        e_nu = rng.uniform(-0.3, 0.3, size=4)
        int_e = rng.uniform(-2.0, 2.0, size=4) # Randomized integral history
        
        eta = ref["eta_d"] + e_eta
        eta[3] = (eta[3] + np.pi) % (2.0 * np.pi) - np.pi
        
        nu_est = rng.uniform(-0.4, 0.4, size=4)
        X_temp = np.hstack((eta, nu_est))
        nu_d = teacher.desired_body_velocity(X_temp, ref)
        nu = nu_d + e_nu
        X = np.hstack((eta, nu))

        rows.append(build_features_and_target(t, X, teacher, shape, int_e))
    return rows


def generate_data():
    os.makedirs("data", exist_ok=True)
    model = BlueROV2Model4DOF()
    teacher = BacksteppingController(model)

    print("--- NEMESIS INN Memory-Aware Data Collection ---")
    rows = []
    for shape in ['spline', 'circle', 'figure8']:
        print(f"Collecting rollout: {shape}")
        rows += generate_rollout_data(model, teacher, shape)

    print(f"Collecting {60000} memory-perturbed samples...")
    rows += generate_random_perturbation_data(model, teacher, n_samples=60000)

    columns = [f"f{i}" for i in range(20)] + ["tau_u", "tau_v", "tau_w", "tau_r"]
    df = pd.DataFrame(rows, columns=columns)
    df.to_csv("data/inn_training_data.csv", index=False)
    print(f"Done. Samples: {len(df)}")


if __name__ == "__main__":
    generate_data()
