import sys
import os
# Add the ROS 2 package to path so we can import models and controllers
sys.path.append(os.path.join(os.path.expanduser('~'), 'bluerov2_control_project', 'ros2_ws', 'src', 'bluerov2_control'))
import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.integrate import solve_ivp

from bluerov2_control.model import BlueROV2Model4DOF
from bluerov2_control.trajectory import reference_trajectory
from bluerov2_control.controllers.observer_linear import ObserverBasedController


def simulate(disturbance_enabled=True, T_final=60.0, trajectory_type="spline"):
    model = BlueROV2Model4DOF()
    controller = ObserverBasedController(model, trajectory_shape=trajectory_type, T_final=T_final)

    t0 = 0.0
    tf = T_final
    dt = 0.02
    t_eval = np.arange(t0, tf + dt, dt)

    ref0 = reference_trajectory(0.0, shape=trajectory_type, T_final=T_final)
    eta0 = ref0["eta_d"]
    nu0 = controller.desired_body_velocity(np.zeros(8), ref0)

    X0 = np.hstack((eta0, nu0))

    # Start observer with small initial estimation mismatch
    Xhat0 = X0.copy()
    #Xhat0[4:8] = np.zeros(4)
    
    Z0 = np.hstack((X0, Xhat0))

    def closed_loop_dynamics(t, Z):
        X = Z[0:8]
        Xhat = Z[8:16]

        ref = reference_trajectory(t, shape=trajectory_type, T_final=T_final)

        # Measurement: only eta = [x, y, z, psi]
        y_meas = X[0:4].copy()

        tau = controller.compute_control(t, Xhat, ref)

        X_dot = model.dynamics(t, X, tau, disturbance_enabled=disturbance_enabled)
        Xhat_dot = controller.observer_dynamics(t, Xhat, y_meas, tau)

        return np.hstack((X_dot, Xhat_dot))

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
    Xhat = Z[:, 8:16]

    refs = [reference_trajectory(ti, shape=trajectory_type, T_final=T_final) for ti in t]
    eta_d = np.array([ref["eta_d"] for ref in refs])

    eta = X[:, 0:4]
    eta_hat = Xhat[:, 0:4]

    e_eta = eta - eta_d
    e_eta[:, 3] = (e_eta[:, 3] + np.pi) % (2.0 * np.pi) - np.pi

    pos_error = np.linalg.norm(e_eta[:, 0:3], axis=1)
    yaw_error = np.abs(e_eta[:, 3])

    estimation_error = X - Xhat
    estimation_error[:, 3] = (estimation_error[:, 3] + np.pi) % (2.0 * np.pi) - np.pi
    estimation_norm = np.linalg.norm(estimation_error, axis=1)

    tau = np.zeros((len(t), 4))
    for i, ti in enumerate(t):
        tau[i, :] = controller.compute_control(ti, Xhat[i, :], reference_trajectory(ti, shape=trajectory_type, T_final=T_final))

    e_rms = np.sqrt(np.mean(pos_error**2))
    e_max = np.max(pos_error)
    est_rms = np.sqrt(np.mean(estimation_norm**2))
    control_effort = np.trapz(np.sum(tau**2, axis=1), t)

    return {
        "t": t,
        "X": X,
        "Xhat": Xhat,
        "eta": eta,
        "eta_hat": eta_hat,
        "eta_d": eta_d,
        "e_eta": e_eta,
        "pos_error": pos_error,
        "yaw_error": yaw_error,
        "estimation_error": estimation_error,
        "estimation_norm": estimation_norm,
        "tau": tau,
        "e_rms": e_rms,
        "e_max": e_max,
        "est_rms": est_rms,
        "control_effort": control_effort,
    }


def save_results_csv(results, filename):
    t = results["t"]
    X = results["X"]
    Xhat = results["Xhat"]
    eta_d = results["eta_d"]
    e_eta = results["e_eta"]
    tau = results["tau"]

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
        "x_hat": Xhat[:, 0],
        "y_hat": Xhat[:, 1],
        "z_hat": Xhat[:, 2],
        "psi_hat": Xhat[:, 3],
        "u_hat": Xhat[:, 4],
        "v_hat": Xhat[:, 5],
        "w_hat": Xhat[:, 6],
        "r_hat": Xhat[:, 7],
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
    ax.set_title(f"Observer Linear 4-DOF Tracking ({mode_name})")
    ax.legend()
    fig.tight_layout()
    fig.savefig(f"figures/observer_linear_{mode_name.lower()}_01_3d_trajectory.pdf", format="pdf", dpi=300, bbox_inches="tight")
    plt.close(fig)

    # 2. XY Tracking
    fig, ax = plt.subplots(figsize=(8.0, 5.5))
    ax.plot(eta_d[:, 0], eta_d[:, 1], "--", linewidth=2.5, color="gray", label="Reference")
    ax.plot(eta[:, 0], eta[:, 1], linewidth=2.0, color="blue", label=f"{mode_name} Path")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"Observer Linear XY Tracking ({mode_name})")
    ax.legend()
    ax.axis("equal")
    fig.tight_layout()
    fig.savefig(f"figures/observer_linear_{mode_name.lower()}_02_xy_tracking.pdf", format="pdf", dpi=300, bbox_inches="tight")
    plt.close(fig)

    # 3. Error Comparison
    fig, ax = plt.subplots(figsize=(8.0, 5.0))
    ax.plot(t, pos_error, linewidth=2.0, color="red", label=f"{mode_name} Error (RMS={result['e_rms']:.4f}m)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("3D Position Error (m)")
    ax.set_title(f"Observer Linear Tracking Error ({mode_name})")
    ax.legend()
    fig.tight_layout()
    fig.savefig(f"figures/observer_linear_{mode_name.lower()}_03_error.pdf", format="pdf", dpi=300, bbox_inches="tight")
    plt.close(fig)

    # 4. Control Inputs
    fig, ax = plt.subplots(figsize=(8.0, 5.2))
    ax.plot(t, tau[:, 0], linewidth=1.8, label=r"$\tau_u$")
    ax.plot(t, tau[:, 1], linewidth=1.8, label=r"$\tau_v$")
    ax.plot(t, tau[:, 2], linewidth=1.8, label=r"$\tau_w$")
    ax.plot(t, tau[:, 3], linewidth=1.8, label=r"$\tau_r$")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Control Force/Torque (N, Nm)")
    ax.set_title(f"Observer Linear Control Inputs | J={effort:.2f} ({mode_name})")
    ax.legend()
    fig.tight_layout()
    fig.savefig(f"figures/observer_linear_{mode_name.lower()}_04_control.pdf", format="pdf", dpi=300, bbox_inches="tight")
    plt.close(fig)

    # 5. Estimation
    fig, ax = plt.subplots(figsize=(8.0, 5.2))
    ax.plot(t, result["estimation_norm"], linewidth=2.0, color="purple", label=f"Est Error (RMS={result['est_rms']:.4f})")
    ax.set_ylabel("Estimation Error Norm")
    ax.set_title(f"Observer Linear State Estimation Error ({mode_name})")
    ax.set_xlabel("Time (s)")
    ax.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(f"figures/observer_linear_{mode_name.lower()}_05_estimation.pdf", format="pdf", dpi=300, bbox_inches="tight")
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

    save_results_csv(nominal, "results/linear_observer_nominal.csv")
    save_results_csv(disturbed, "results/linear_observer_disturbed.csv")

    plot_results(nominal, "Nominal")
    plot_results(disturbed, "Disturbed")

    summary = pd.DataFrame([
        {
            "controller": "Observer-Based",
            "case": "nominal",
            "e_rms_m": nominal["e_rms"],
            "e_max_m": nominal["e_max"],
            "est_rms": nominal["est_rms"],
            "control_effort": nominal["control_effort"],
        },
        {
            "controller": "Observer-Based",
            "case": "disturbed",
            "e_rms_m": disturbed["e_rms"],
            "e_max_m": disturbed["e_max"],
            "est_rms": disturbed["est_rms"],
            "control_effort": disturbed["control_effort"],
        },
    ])

    summary.to_csv("results/linear_observer_summary.csv", index=False)

    print("\nObserver-Based Simulation Complete")
    print(summary)
    print("\nOnly 4 journal-style figures saved in: figures/")
    print("CSV results saved in: results/")


if __name__ == "__main__":
    main()
