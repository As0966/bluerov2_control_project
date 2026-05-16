import sys
import os
# Add the ROS 2 package to path so we can import models and controllers
sys.path.insert(0, os.path.join(os.path.expanduser('~'), 'bluerov2_control_project', 'ros2_ws', 'src', 'bluerov2_control'))
import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.integrate import solve_ivp

from bluerov2_control.model import BlueROV2Model4DOF
from bluerov2_control.trajectory import reference_trajectory
from bluerov2_control.controllers.mrac import MRACController


def simulate(disturbance_enabled=True, T_final=60.0, trajectory_type="spline"):
    model = BlueROV2Model4DOF()
    controller = MRACController(model, trajectory_shape=trajectory_type, T_final=T_final)

    t0 = 0.0
    tf = T_final
    dt = 0.02
    t_eval = np.arange(t0, tf + dt, dt)

    ref0 = reference_trajectory(0.0, shape=trajectory_type, T_final=T_final)
    eta0 = ref0["eta_d"]

    J0 = model.rotation_matrix(eta0[3])
    nu0 = np.linalg.inv(J0) @ ref0["eta_dot_d"]

    X0 = np.hstack((eta0, nu0))

    # Initial reference model velocity
    nu_m0 = nu0.copy()

    # Start with imperfect adaptive parameters
    theta1_0 = 0.65 * controller.theta1_nominal
    theta2_0 = 0.65 * controller.theta2_nominal
    theta3_0 = np.zeros(4)

    Z0 = np.hstack((X0, nu_m0, theta1_0, theta2_0, theta3_0))

    def closed_loop_dynamics(t, Z):
        X = Z[0:8]
        nu_m = Z[8:12]
        theta1 = Z[12:16]
        theta2 = Z[16:20]
        theta3 = Z[20:24]

        ref = reference_trajectory(t, shape=trajectory_type, T_final=T_final)

        eta = X[0:4]
        nu = X[4:8]
        nu_d = controller.desired_body_velocity(X, ref)

        tau = controller.compute_control(X, nu, nu_d, theta1, theta2, theta3)

        X_dot = model.dynamics(
            t,
            X,
            tau,
            disturbance_enabled=disturbance_enabled,
        )

        nu_m_dot = controller.reference_model_dynamics(nu_m, nu_d)

        theta1_dot, theta2_dot, theta3_dot = controller.adaptation_dynamics(
            nu,
            nu_d,
            nu_m,
            theta1,
            theta2,
            theta3,
        )

        return np.hstack((X_dot, nu_m_dot, theta1_dot, theta2_dot, theta3_dot))

    sol = solve_ivp(
        closed_loop_dynamics,
        (t0, tf),
        Z0,
        t_eval=t_eval,
        method="RK45",
        rtol=1e-6,
        atol=1e-8,
    )

    t = sol.t
    Z = sol.y.T

    X = Z[:, 0:8]
    nu_m = Z[:, 8:12]
    theta1 = Z[:, 12:16]
    theta2 = Z[:, 16:20]
    theta3 = Z[:, 20:24]

    refs = [reference_trajectory(ti, shape=trajectory_type, T_final=T_final) for ti in t]
    eta_d = np.array([ref["eta_d"] for ref in refs])

    eta = X[:, 0:4]
    nu = X[:, 4:8]

    e_eta = eta - eta_d
    e_eta[:, 3] = (e_eta[:, 3] + np.pi) % (2.0 * np.pi) - np.pi

    pos_error = np.linalg.norm(e_eta[:, 0:3], axis=1)
    yaw_error = np.abs(e_eta[:, 3])

    tau = np.zeros((len(t), 4))
    nu_d_log = np.zeros((len(t), 4))

    for i, ti in enumerate(t):
        ref = refs[i]
        nu_d = controller.desired_body_velocity(X[i, :], ref)
        nu_d_log[i, :] = nu_d
        tau[i, :] = controller.compute_control(
            X[i, :],
            nu[i, :],
            nu_d,
            theta1[i, :],
            theta2[i, :],
            theta3[i, :],
        )

    e_rms = np.sqrt(np.mean(pos_error**2))
    e_max = np.max(pos_error)
    control_effort = np.trapz(np.sum(tau**2, axis=1), t)

    return {
        "t": t,
        "X": X,
        "eta": eta,
        "nu": nu,
        "nu_m": nu_m,
        "nu_d": nu_d_log,
        "eta_d": eta_d,
        "e_eta": e_eta,
        "pos_error": pos_error,
        "yaw_error": yaw_error,
        "tau": tau,
        "theta1": theta1,
        "theta2": theta2,
        "theta3": theta3,
        "e_rms": e_rms,
        "e_max": e_max,
        "control_effort": control_effort,
    }


def save_results_csv(results, filename):
    t = results["t"]
    X = results["X"]
    eta_d = results["eta_d"]
    e_eta = results["e_eta"]
    tau = results["tau"]
    theta1 = results["theta1"]
    theta2 = results["theta2"]
    theta3 = results["theta3"]

    df = pd.DataFrame({
        "t": t,
        "x": X[:, 0],
        "y": X[:, 1],
        "z": X[:, 2],
        "psi": X[:, 3],
        "u": X[:, 4],
        "v": X[:, 5],
        "w": X[:, 6],
        "r": X[:, 7],
        "x_d": eta_d[:, 0],
        "y_d": eta_d[:, 1],
        "z_d": eta_d[:, 2],
        "psi_d": eta_d[:, 3],
        "e_x": e_eta[:, 0],
        "e_y": e_eta[:, 1],
        "e_z": e_eta[:, 2],
        "e_psi": e_eta[:, 3],
        "tau_u": tau[:, 0],
        "tau_v": tau[:, 1],
        "tau_w": tau[:, 2],
        "tau_r": tau[:, 3],
        "theta1_u": theta1[:, 0],
        "theta1_v": theta1[:, 1],
        "theta1_w": theta1[:, 2],
        "theta1_r": theta1[:, 3],
        "theta2_u": theta2[:, 0],
        "theta2_v": theta2[:, 1],
        "theta2_w": theta2[:, 2],
        "theta2_r": theta2[:, 3],
        "theta3_u": theta3[:, 0],
        "theta3_v": theta3[:, 1],
        "theta3_w": theta3[:, 2],
        "theta3_r": theta3[:, 3],
    })

    df.to_csv(filename, index=False)


def plot_results(result, mode_name):
    os.makedirs("figures", exist_ok=True)
    
    import matplotlib.pyplot as plt
    # Use seaborn-style if available for cleaner plots
    try:
        plt.style.use("seaborn-v0_8-whitegrid")
    except:
        pass

    t = result["t"]
    eta_d = result["eta_d"]
    eta = result["eta"]
    pos_error = result["pos_error"]
    tau = result["tau"]
    effort = result["control_effort"]

    # 1. 3D Trajectory
    fig = plt.figure(figsize=(8.5, 6.5))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(eta_d[:, 0], eta_d[:, 1], eta_d[:, 2], "--", linewidth=2.5, color="gray", label="Reference")
    ax.plot(eta[:, 0], eta[:, 1], eta[:, 2], linewidth=2.0, color="blue", label=f"{mode_name} Path")
    ax.set_xlabel("X (m)", labelpad=10)
    ax.set_ylabel("Y (m)", labelpad=10)
    ax.set_zlabel("Z (m)", labelpad=10)
    ax.set_title(f"MRAC 4-DOF Tracking ({mode_name})")
    ax.legend()
    fig.tight_layout()
    fig.savefig(f"figures/mrac_{mode_name.lower()}_01_3d_trajectory.pdf", format="pdf", dpi=300, bbox_inches="tight")
    plt.close(fig)

    # 2. XY Tracking
    fig, ax = plt.subplots(figsize=(8.0, 5.5))
    ax.plot(eta_d[:, 0], eta_d[:, 1], "--", linewidth=2.5, color="gray", label="Reference")
    ax.plot(eta[:, 0], eta[:, 1], linewidth=2.0, color="blue", label=f"{mode_name} Path")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"MRAC XY Tracking ({mode_name})")
    ax.legend()
    ax.axis("equal")
    fig.tight_layout()
    fig.savefig(f"figures/mrac_{mode_name.lower()}_02_xy_tracking.pdf", format="pdf", dpi=300, bbox_inches="tight")
    plt.close(fig)

    # 3. Error Comparison
    fig, ax = plt.subplots(figsize=(8.0, 5.0))
    ax.plot(t, pos_error, linewidth=2.0, color="red", label=f"{mode_name} Error (RMS={result['e_rms']:.4f}m)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("3D Position Error (m)")
    ax.set_title(f"MRAC Tracking Error ({mode_name})")
    ax.legend()
    fig.tight_layout()
    fig.savefig(f"figures/mrac_{mode_name.lower()}_03_error.pdf", format="pdf", dpi=300, bbox_inches="tight")
    plt.close(fig)

    # 4. Control Inputs
    fig, ax = plt.subplots(figsize=(8.0, 5.2))
    ax.plot(t, tau[:, 0], linewidth=1.8, label=r"$\tau_u$")
    ax.plot(t, tau[:, 1], linewidth=1.8, label=r"$\tau_v$")
    ax.plot(t, tau[:, 2], linewidth=1.8, label=r"$\tau_w$")
    ax.plot(t, tau[:, 3], linewidth=1.8, label=r"$\tau_r$")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Control Force/Torque (N, Nm)")
    ax.set_title(f"MRAC Control Inputs | J={effort:.2f} ({mode_name})")
    ax.legend()
    fig.tight_layout()
    fig.savefig(f"figures/mrac_{mode_name.lower()}_04_control.pdf", format="pdf", dpi=300, bbox_inches="tight")
    plt.close(fig)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--trajectory', type=str, default='spline', choices=['spline', 'circle', 'figure8'])
    parser.add_argument('--duration', type=float, default=60.0)
    args = parser.parse_args()

    os.makedirs("results", exist_ok=True)
    os.makedirs("figures", exist_ok=True)

    nominal = simulate(disturbance_enabled=False, trajectory_type=args.trajectory, T_final=args.duration)
    disturbed = simulate(disturbance_enabled=True, trajectory_type=args.trajectory, T_final=args.duration)

    save_results_csv(nominal, "results/mrac_nominal.csv")
    save_results_csv(disturbed, "results/mrac_disturbed.csv")

    plot_results(nominal, "Nominal")
    plot_results(disturbed, "Disturbed")

    summary = pd.DataFrame([
        {
            "controller": "MRAC",
            "case": "nominal",
            "e_rms_m": nominal["e_rms"],
            "e_max_m": nominal["e_max"],
            "control_effort": nominal["control_effort"],
        },
        {
            "controller": "MRAC",
            "case": "disturbed",
            "e_rms_m": disturbed["e_rms"],
            "e_max_m": disturbed["e_max"],
            "control_effort": disturbed["control_effort"],
        },
    ])

    summary.to_csv("results/mrac_summary.csv", index=False)

    print("\nMRAC Simulation Complete")
    print(summary)
    print("\nFigures saved in: figures/")
    print("CSV results saved in: results/")


if __name__ == "__main__":
    main()
