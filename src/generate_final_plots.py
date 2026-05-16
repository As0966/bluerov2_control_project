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
from bluerov2_control.controllers.backstepping import BacksteppingController
from bluerov2_control.controllers.adaptive_backstepping import AdaptiveBacksteppingController
from bluerov2_control.controllers.mrac import MRACController
from bluerov2_control.controllers.inn import INNController
from bluerov2_control.controllers.state_feedback import StateFeedbackController

def run_mission(controller_type="backstepping", trajectory="figure8", T_final=60.0):
    actual_model = BlueROV2Model4DOF()
    actual_model.m_u *= 1.25 # 25% mass mismatch
    actual_model.m_v *= 1.25
    actual_model.I_z *= 1.30
    nominal_model = BlueROV2Model4DOF()

    if controller_type == "adaptive":
        controller = AdaptiveBacksteppingController(nominal_model, trajectory_shape=trajectory, T_final=T_final)
    elif controller_type == "mrac":
        controller = MRACController(nominal_model, trajectory_shape=trajectory, T_final=T_final)
        mrac_state = {'nu_m': np.zeros(4), 't1': 0.65*controller.theta1_nominal, 't2': 0.65*controller.theta2_nominal, 't3': np.zeros(4)}
    elif controller_type == "inn":
        controller = INNController(nominal_model, trajectory_shape=trajectory, T_final=T_final)
    elif controller_type == "state_feedback":
        controller = StateFeedbackController(nominal_model, trajectory_shape=trajectory, T_final=T_final)
    else:
        controller = BacksteppingController(nominal_model, trajectory_shape=trajectory, T_final=T_final)

    t0 = 0.0; tf = T_final; dt = 0.01
    t_eval = np.arange(t0, tf + dt, dt)
    ref0 = reference_trajectory(0.0, shape=trajectory, T_final=T_final)
    X0 = np.hstack((ref0["eta_d"], np.zeros(4)))

    tau_history = []
    
    def dynamics(t, X):
        X_noisy = X + np.random.normal(0, 0.002, size=8)
        ref = reference_trajectory(t, shape=trajectory, T_final=T_final)
        nu = X_noisy[4:8]
        
        if controller_type == "adaptive":
            nu_d = controller.desired_body_velocity(X_noisy, ref)
            controller.update_adaptation(X_noisy, nu_d, np.zeros(4), 0.01)
            tau = controller.compute_control(t, X_noisy, ref)
        elif controller_type == "mrac":
            nu_d = controller.desired_body_velocity(X_noisy, ref)
            tau = controller.compute_control(X_noisy, nu, nu_d, mrac_state['t1'], mrac_state['t2'], mrac_state['t3'])
            nu_m_next, t1n, t2n, t3n = controller.update(t, X_noisy, mrac_state['nu_m'], mrac_state['t1'], mrac_state['t2'], mrac_state['t3'], 0.01)
            mrac_state['nu_m'], mrac_state['t1'], mrac_state['t2'], mrac_state['t3'] = nu_m_next, t1n, t2n, t3n
        else:
            tau = controller.compute_control(t, X_noisy, ref)
            
        tau_history.append(tau.copy())
        return actual_model.dynamics(t, X, tau, disturbance_enabled=True)

    sol = solve_ivp(dynamics, (t0, tf), X0, t_eval=t_eval, method="RK45")
    t = sol.t; X = sol.y.T
    eta_d = np.array([reference_trajectory(ti, shape=trajectory, T_final=T_final)["eta_d"] for ti in t])
    pos_error = np.linalg.norm(X[:, 0:3] - eta_d[:, 0:3], axis=1)
    tau_arr = np.array(tau_history[:len(t)])
    
    return {"t": t, "X": X, "eta_d": eta_d, "pos_error": pos_error, "tau": tau_arr}

def generate_visual_portfolio():
    os.makedirs("portfolio", exist_ok=True)
    for traj in ['circle', 'figure8']:
        print(f"--- Generating Dashboard for {traj} ---")
        # For the dashboard, we use the BEST controller (MRAC or Backstepping)
        # to show what the proposal aims to achieve.
        res = run_mission("mrac", trajectory=traj)
        
        fig, axs = plt.subplots(4, 1, figsize=(10, 15))
        
        # 1. Trajectory
        axs[0].plot(res["eta_d"][:, 0], res["eta_d"][:, 1], 'k--', label='Ref', alpha=0.5)
        axs[0].plot(res["X"][:, 0], res["X"][:, 1], 'b-', label='MRAC Path')
        axs[0].set_title(f"NEMESIS Dashboard: {traj.capitalize()} Path")
        axs[0].legend(); axs[0].axis('equal'); axs[0].grid(True, alpha=0.3)
        
        # 2. Depth
        axs[1].plot(res["t"], res["eta_d"][:, 2], 'k--', label='Ref Z', alpha=0.5)
        axs[1].plot(res["t"], res["X"][:, 2], 'g-', label='Actual Z')
        axs[1].set_title("Precision Depth Tracking")
        axs[1].grid(True, alpha=0.3); axs[1].legend()
        
        # 3. Control Effort
        axs[2].plot(res["t"], res["tau"][:, 0], label='Surge')
        axs[2].plot(res["t"], res["tau"][:, 1], label='Sway')
        axs[2].plot(res["t"], res["tau"][:, 2], label='Heave')
        axs[2].set_title("Actuator Efficiency (Control Effort)")
        axs[2].grid(True, alpha=0.3); axs[2].legend()
        
        # 4. J-Cost
        axs[3].plot(res["t"], res["pos_error"], 'r-', label='J (Tracking Error)')
        axs[3].fill_between(res["t"], 0, res["pos_error"], color='red', alpha=0.1)
        axs[3].set_title(f"J-Cost Convergence (Mean: {np.mean(res['pos_error']):.4f}m)")
        axs[3].set_xlabel("Time (s)"); axs[3].grid(True, alpha=0.3); axs[3].legend()
        
        plt.tight_layout()
        plt.savefig(f"portfolio/dashboard_{traj}.png", dpi=300)
        plt.close()
    
    print("\nFull Research Dashboards Generated in portfolio/ folder.")

if __name__ == "__main__":
    generate_visual_portfolio()
