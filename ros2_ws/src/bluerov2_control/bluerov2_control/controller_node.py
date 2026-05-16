import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Pose, Twist
from nav_msgs.msg import Odometry
import numpy as np
import os
import pandas as pd
import matplotlib.pyplot as plt

from .model import BlueROV2Model4DOF
from .thruster_allocator import ThrusterAllocator
from .trajectory import reference_trajectory
from .controllers.backstepping import BacksteppingController
from .controllers.observer_linear import ObserverBasedController

# HASTEN-Compliant Controllers
from .controllers.state_feedback_hasten import HastenStateFeedbackController
from .controllers.feedback_linearization_input import HastenInputFeedbackLinearizationController
from .controllers.feedback_linearization_io_hasten import HastenIOFeedbackLinearizationController
from .controllers.mrac_hasten import HastenMRACController
from .controllers.adaptive_backstepping_hasten import HastenAdaptiveBacksteppingController
from .controllers.inn_hasten import HastenINNController

def quaternion_to_yaw(q):
    """Convert a quaternion to yaw angle (ENU)."""
    x, y, z, w = q
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)

def compute_disturbance(t: float, enabled: bool) -> np.ndarray:
    if not enabled:
        return np.zeros(4)
    return np.array([
        0.8 * np.sin(0.2 * t),
        0.5 * np.cos(0.15 * t),
        0.4 * np.sin(0.1 * t),
        0.2 * np.sin(0.12 * t),
    ])

SIMPLE_CONTROLLERS = {
    'backstepping': BacksteppingController,
    'hasten_state_feedback': HastenStateFeedbackController,
    'hasten_fl_input': HastenInputFeedbackLinearizationController,
    'hasten_fl_io': HastenIOFeedbackLinearizationController,
}

OBSERVER_CONTROLLERS = {
    'observer_linear': ObserverBasedController,
}

ADAPTIVE_CONTROLLERS = {
    'hasten_mrac': HastenMRACController,
    'hasten_adaptive_backstepping': HastenAdaptiveBacksteppingController,
    'hasten_inn': HastenINNController,
}

class BlueROV2ControllerNode(Node):
    CONTROL_RATE = 100.0  # Hz

    def __init__(self):
        super().__init__('bluerov2_controller')

        self.declare_parameter('controller', 'backstepping')
        self.declare_parameter('duration', 80.0)
        self.declare_parameter('disturbance', False)
        self.declare_parameter('trajectory', 'spline')
        
        self.ctrl_name = str(self.get_parameter('controller').value)
        self.duration = float(self.get_parameter('duration').value)
        self.disturbance_enabled = bool(self.get_parameter('disturbance').value)
        self.trajectory_shape = str(self.get_parameter('trajectory').value)

        self.model = BlueROV2Model4DOF()
        self.allocator = ThrusterAllocator()

        self.controller = self._create_controller(self.ctrl_name)
        self.get_logger().info(f'Controller: {self.ctrl_name} | Trajectory: {self.trajectory_shape}')

        self.Xhat = np.zeros(8)
        self.delta_hat = np.zeros(4)
        self.bias_hat = np.zeros(4)
        self.nu_m = np.zeros(4)
        self.theta1 = None
        self.theta2 = None
        self.theta3 = None
        self.W_inn = None
        self.theta_bs = None
        self.z_eta_bs = None

        if self.ctrl_name == 'hasten_mrac':
            self.theta1 = self.controller.theta1_nominal.copy()
            self.theta2 = self.controller.theta2_nominal.copy()
        elif self.ctrl_name == 'hasten_adaptive_backstepping':
            self.theta_bs = self.controller.theta_nominal.copy()
        elif self.ctrl_name == 'hasten_inn':
            pass # Uses internal W

        self.X = np.zeros(8)
        self.odom_received = False
        self.sim_time = 0.0
        self.start_time = None
        self.dt = 1.0 / self.CONTROL_RATE

        # Logging history
        self.hist_t = []
        self.hist_X = []
        self.hist_Xhat = []
        self.hist_eta_d = []
        self.hist_tau = []
        self.hist_delta_hat = []
        self.hist_true_delta = []
        self.shutdown_triggered = False

        self.odom_sub = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.wrench_pub = self.create_publisher(Wrench, '/model/bluerov2/body_wrench', 10)
        self.timer = self.create_timer(self.dt, self._control_loop)

        self.get_logger().info(f'BlueROV2 controller node started @ {self.CONTROL_RATE} Hz')

    def wrap_angle(self, angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def _create_controller(self, name):
        all_controllers = {**SIMPLE_CONTROLLERS, **OBSERVER_CONTROLLERS, **ADAPTIVE_CONTROLLERS}
        if name not in all_controllers:
            name = 'backstepping'
        return all_controllers[name](self.model, trajectory_shape=self.trajectory_shape, T_final=self.duration)

    def _odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        vel = msg.twist.twist
        
        yaw_enu = quaternion_to_yaw([ori.x, ori.y, ori.z, ori.w])
        psi_ned = self.wrap_angle(np.pi/2.0 - yaw_enu)

        # Reverting to the RAW velocity mapping (assuming body-frame from Gazebo)
        u_ned = vel.linear.x
        v_ned = -vel.linear.y # NED Y is Right, GZ Y is Left
        w_ned = -vel.linear.z # NED Z is Down, GZ Z is Up
        r_ned = -vel.angular.z # NED Yaw is CW, GZ Yaw is CCW

        self.yaw_enu = yaw_enu
        self.X = np.array([pos.y, pos.x, -pos.z, psi_ned, u_ned, v_ned, w_ned, r_ned])

        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not self.odom_received:
            self.odom_received = True
            self.start_time = self.sim_time
            self.Xhat = self.X.copy()
            self.get_logger().info(f'Odom received.')

    def _control_loop(self):
        if not self.odom_received or self.shutdown_triggered:
            return

        t = self.sim_time - self.start_time

        # 1. Calculate Time Since Last Odom (Latency Compensation)
        dt_delay = self.sim_time - self.start_time - t
        if dt_delay < 0: dt_delay = 0.0
        
        # 2. Project State Forward (Kinematic Prediction)
        X = self.X.copy()
        eta = X[0:4]
        nu = X[4:8]
        J = self.model.rotation_matrix(eta[3])
        eta_pred = eta + (J @ nu) * dt_delay
        eta_pred[3] = self.wrap_angle(eta_pred[3])
        X_pred = np.hstack((eta_pred, nu))
        
        # 3. Compute Control on Predicted State
        ref = reference_trajectory(t, shape=self.trajectory_shape, T_final=self.duration)
        
        tau = self._compute_tau(t, X_pred, ref)
        
        # Check for NaN or Inf to prevent Gazebo crash
        if not np.all(np.isfinite(tau)):
            self.get_logger().error(f"Controller {self.ctrl_name} produced non-finite tau: {tau}. Resetting to zero.")
            tau = np.zeros(4)
        true_delta = compute_disturbance(t, self.disturbance_enabled)

        self.hist_t.append(t)
        self.hist_X.append(X)
        self.hist_Xhat.append(self.Xhat.copy())
        self.hist_eta_d.append(ref['eta_d'])
        self.hist_tau.append(tau)
        self.hist_delta_hat.append(self.delta_hat.copy())
        self.hist_true_delta.append(true_delta)

        # Convert Tau (NED) back to Gazebo WORLD frame (ENU)
        total_wrench = tau + true_delta
        f_u = total_wrench[0]
        f_v_left = -total_wrench[1] # NED Y is Right, GZ Y is Left
        
        cos_y = np.cos(self.yaw_enu)
        
        # 4. Transform Body-Frame Tau to World-Frame Wrench for Gazebo
        # Gazebo's body_wrench often expects World Frame or has ENU axes
        eta = self.X[0:4]
        psi = eta[3]
        
        # J_rot for 3D position (no yaw in world force)
        c, s = np.cos(psi), np.sin(psi)
        R = np.array([
            [c, -s, 0],
            [s,  c, 0],
            [0,  0, 1]
        ])
        
        # tau is [F_surge, F_sway, F_heave, T_yaw] in NED Body
        f_body = total_wrench[0:3]
        f_world_ned = R @ f_body
        
        # Map NED World back to ENU World for Gazebo
        # NED [N, E, D] -> ENU [E, N, U]
        f_gz_enu = [f_world_ned[1], f_world_ned[0], -f_world_ned[2]]
        t_gz_enu = [0.0, 0.0, -total_wrench[3]] # NED Yaw (CW) -> ENU Yaw (CCW)
        
        msg = Wrench()
        msg.force.x = float(f_gz_enu[0])
        msg.force.y = float(f_gz_enu[1])
        msg.force.z = float(f_gz_enu[2])
        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = float(t_gz_enu[2])
        
        self.wrench_pub.publish(msg)

        if t >= self.duration:
            self.shutdown_triggered = True
            self._save_results()
            raise SystemExit

    def _compute_tau(self, t, X, ref):
        if self.ctrl_name in SIMPLE_CONTROLLERS:
            return self.controller.compute_control(t, X)
        elif self.ctrl_name == 'observer_linear':
            y_meas = X[0:4]
            tau = self.controller.compute_control(t, self.Xhat, ref)
            self.Xhat, self.bias_hat = self.controller.observer_update(self.Xhat, y_meas, tau, self.dt, bias_hat=self.bias_hat)
            return tau
        elif self.ctrl_name == 'hasten_mrac':
            nu = X[4:8]
            nu_d = self.controller.desired_body_velocity(X, ref)
            tau = self.controller.compute_control(X, nu, nu_d, self.theta1, self.theta2)
            self.nu_m, self.theta1, self.theta2 = self.controller.update(t, X, self.nu_m, self.theta1, self.theta2, self.dt)
            return tau
        elif self.ctrl_name == 'hasten_adaptive_backstepping':
            tau = self.controller.compute_control(t, X, self.theta_bs, ref)
            self.theta_bs = self.controller.update(t, X, self.theta_bs, self.dt)
            return tau
        elif self.ctrl_name == 'hasten_inn':
            tau = self.controller.compute_control(t, X, ref)
            self.controller.update(t, X, self.dt)
            return tau
        return np.zeros(4)

    def _save_results(self):
        if not self.hist_t:
            return
            
        t_arr = np.array(self.hist_t)
        X_arr = np.array(self.hist_X)
        eta_d_arr = np.array(self.hist_eta_d)
        tau_arr = np.array(self.hist_tau)
        
        # 1. Save CSV Data
        df = pd.DataFrame({
            't': t_arr,
            'x': X_arr[:,0], 'y': X_arr[:,1], 'z': X_arr[:,2], 'psi': X_arr[:,3],
            'u': X_arr[:,4], 'v': X_arr[:,5], 'w': X_arr[:,6], 'r': X_arr[:,7],
            'x_d': eta_d_arr[:,0], 'y_d': eta_d_arr[:,1], 'z_d': eta_d_arr[:,2], 'psi_d': eta_d_arr[:,3],
            'tau_u': tau_arr[:,0], 'tau_v': tau_arr[:,1], 'tau_w': tau_arr[:,2], 'tau_r': tau_arr[:,3]
        })
        os.makedirs('results', exist_ok=True)
        csv_filename = f"{self.ctrl_name}_{self.trajectory_shape}_gazebo.csv"
        df.to_csv(os.path.join('results', csv_filename), index=False)
        self.get_logger().info(f'Saved telemetry to results/{csv_filename}')

        # 2. Generate and Save Comprehensive Research Plot
        try:
            os.makedirs('figures', exist_ok=True)
            fig, axs = plt.subplots(4, 1, figsize=(10, 15))
            
            # Subplot 1: XY Trajectory
            axs[0].plot(eta_d_arr[:,0], eta_d_arr[:,1], 'k--', label='Reference', alpha=0.5)
            axs[0].plot(X_arr[:,0], X_arr[:,1], 'b-', label='Actual Path')
            axs[0].set_title(f"Trajectory Profile: {self.ctrl_name.upper()}")
            axs[0].set_xlabel("X (m)"); axs[0].set_ylabel("Y (m)")
            axs[0].legend(); axs[0].axis('equal'); axs[0].grid(True, alpha=0.3)
            
            # Subplot 2: Depth (Z) Tracking
            axs[1].plot(t_arr, eta_d_arr[:,2], 'k--', label='Ref Z', alpha=0.5)
            axs[1].plot(t_arr, X_arr[:,2], 'g-', label='Actual Z')
            axs[1].set_title("Depth Profile (Z)")
            axs[1].set_ylabel("Depth (m)"); axs[1].grid(True, alpha=0.3); axs[1].legend()
            
            # Subplot 3: Control Effort (Tau)
            axs[2].plot(t_arr, tau_arr[:,0], label='Surge (u)')
            axs[2].plot(t_arr, tau_arr[:,1], label='Sway (v)')
            axs[2].plot(t_arr, tau_arr[:,2], label='Heave (w)')
            axs[2].set_title("Control Effort (Forces)")
            axs[2].set_ylabel("Force (N)"); axs[2].grid(True, alpha=0.3); axs[2].legend(loc='upper right')
            
            # Subplot 4: J-Cost (3D Euclidean Error)
            error = np.linalg.norm(X_arr[:,0:3] - eta_d_arr[:,0:3], axis=1)
            axs[3].plot(t_arr, error, 'r-', label='J (Tracking Error)')
            axs[3].fill_between(t_arr, 0, error, color='red', alpha=0.1)
            axs[3].set_title(f"J-Cost Performance (Mean: {np.mean(error):.4f}m)")
            axs[3].set_ylabel("Error (m)"); axs[3].set_xlabel("Time (s)")
            axs[3].grid(True, alpha=0.3); axs[3].legend()
            
            plot_filename = f"{self.ctrl_name}_{self.trajectory_shape}_gazebo.png"
            plt.tight_layout()
            plt.savefig(os.path.join('figures', plot_filename), dpi=300)
            plt.close()
            self.get_logger().info(f'Saved Research Dashboard to figures/{plot_filename}')
        except Exception as e:
            self.get_logger().error(f"Failed to save dashboard: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BlueROV2ControllerNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
