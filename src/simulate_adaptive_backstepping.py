import sys
import os
# Add the ROS 2 package to path
sys.path.append(os.path.join(os.path.expanduser('~'), 'bluerov2_control_project', 'ros2_ws', 'src', 'bluerov2_control'))

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.integrate import solve_ivp

from bluerov2_control.model import BlueROV2Model4DOF
from bluerov2_control.trajectory import reference_trajectory
from bluerov2_control.controllers.adaptive_backstepping import AdaptiveBacksteppingController

def simulate(disturbance_enabled=True, T_final=60.0, trajectory_type="spline"):
    model = BlueROV2Model4DOF()
    controller = AdaptiveBacksteppingController(model, trajectory_shape=trajectory_type, T_final=T_final)

    t0 = 0.0
    tf = T_final
    dt = 0.02
    t_eval = np.arange(t0, tf + dt, dt)

    ref0 = reference_trajectory(0.0, shape=trajectory_type, T_final=T_final)
    X0 = np.hstack((ref0["eta_d"], np.zeros(4)))

    # Track internal adaptive parameters
    m_estimates = []
    d_estimates = []

    def closed_loop_dynamics(t, X):
        ref = reference_trajectory(t, shape=trajectory_type, T_final=T_final)
        
        # Calculate feedforward terms for adaptation update
        nu_d = controller.desired_body_velocity(X, ref)
        
        # Numerical derivative for nu_d_dot
        dt_step = 0.01
        ref_next = reference_trajectory(t + dt_step, shape=trajectory_type, T_final=T_final)
        nu_d_next = controller.desired_body_velocity(X, ref_next)
        nu_d_dot = (nu_d_next - nu_d) / dt_step
        
        # Update adaptation (in the simulation loop)
        controller.update_adaptation(X, nu_d, nu_d_dot, 0.01) # Small dt for stability
        
        tau = controller.compute_control(t, X, ref)
        
        # Log estimates
        m_estimates.append(np.diag(controller.M_hat).copy())
        d_estimates.append(np.diag(controller.D_hat).copy())
        
        return model.dynamics(t, X, tau, disturbance_enabled=disturbance_enabled)

    sol = solve_ivp(
        closed_loop_dynamics,
        (t0, tf),
        X0,
        t_eval=t_eval,
        method="RK45",
        rtol=1e-6,
        atol=1e-8,
    )

    t = sol.t
    X = sol.y.T
    
    # Post-process results
    refs = [reference_trajectory(ti, shape=trajectory_type, T_final=T_final) for ti in t]
    eta_d = np.array([ref["eta_d"] for ref in refs])
    eta = X[:, 0:4]
    pos_error = np.linalg.norm(eta[:, 0:3] - eta_d[:, 0:3], axis=1)
    
    tau = np.zeros((len(t), 4))
    for i, ti in enumerate(t):
        tau[i, :] = controller.compute_control(ti, X[i, :], refs[i])

    return {
        "t": t,
        "X": X,
        "eta": eta,
        "eta_d": eta_d,
        "pos_error": pos_error,
        "tau": tau,
        "e_rms": np.sqrt(np.mean(pos_error**2)),
        "control_effort": np.trapz(np.sum(tau**2, axis=1), t)
    }

def plot_results(result, mode_name):
    os.makedirs("figures", exist_ok=True)
    t = result["t"]
    eta = result["eta"]
    eta_d = result["eta_d"]
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(eta_d[:, 0], eta_d[:, 1], eta_d[:, 2], 'r--', label='Reference')
    ax.plot(eta[:, 0], eta[:, 1], eta[:, 2], 'b-', label='Adaptive Backstepping')
    ax.set_title(f"Adaptive Backstepping Tracking ({mode_name})")
    ax.legend()
    plt.savefig(f"figures/adaptive_backstepping_{mode_name.lower()}_3d.png")
    plt.close()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--trajectory', type=str, default='figure8', choices=['spline', 'circle', 'figure8'])
    parser.add_argument('--duration', type=float, default=60.0)
    args = parser.parse_args()

    res = simulate(disturbance_enabled=True, T_final=args.duration, trajectory_type=args.trajectory)
    print(f"Adaptive Backstepping ({args.trajectory}) RMS Error: {res['e_rms']:.4f} m")
    plot_results(res, f"Disturbed_{args.trajectory}")
