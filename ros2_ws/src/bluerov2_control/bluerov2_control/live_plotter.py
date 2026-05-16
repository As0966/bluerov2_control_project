#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

# Use PyQtGraph for fast, non-blocking live plotting (matplotlib plt.pause often blocks ROS spin)
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore
    HAS_PG = True
except ImportError:
    HAS_PG = False
    print("[WARNING] pyqtgraph not found. Live plotter will exit.")
    print("Install it with: pip3 install pyqtgraph PyQt5")

from bluerov2_control.trajectory import reference_trajectory

class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')

        self.declare_parameter('trajectory', 'spline')
        self.declare_parameter('duration', 60.0)
        self.trajectory_shape = str(self.get_parameter('trajectory').value)
        self.duration = float(self.get_parameter('duration').value)

        if not HAS_PG:
            self.get_logger().error("pyqtgraph is required for real-time plotting.")
            return

        # Start PyQt application
        self.app = pg.mkQApp("BlueROV2 Live Plotter")
        
        # Setup Window
        self.win = pg.GraphicsLayoutWidget(show=True, title="BlueROV2 Live Telemetry")
        self.win.resize(1000, 600)
        
        # Setup Plots
        self.plot_xy = self.win.addPlot(title="X-Y Path")
        self.plot_xy.setLabel('bottom', 'X Position', units='m')
        self.plot_xy.setLabel('left', 'Y Position', units='m')
        self.plot_xy.showGrid(x=True, y=True)
        self.plot_xy.setAspectLocked(True)
        
        self.plot_z = self.win.addPlot(title="Depth (Z)")
        self.plot_z.setLabel('bottom', 'Time', units='s')
        self.plot_z.setLabel('left', 'Depth', units='m')
        self.plot_z.showGrid(x=True, y=True)
        
        self.win.nextRow()
        
        self.plot_err = self.win.addPlot(title="3D Tracking Error")
        self.plot_err.setLabel('bottom', 'Time', units='s')
        self.plot_err.setLabel('left', 'Error', units='m')
        self.plot_err.showGrid(x=True, y=True)
        
        # Plot curves
        self.curve_ref_xy = self.plot_xy.plot(pen=pg.mkPen('y', width=2), name="Reference")
        self.curve_act_xy = self.plot_xy.plot(pen=pg.mkPen('c', width=2), name="Actual")
        
        self.curve_ref_z = self.plot_z.plot(pen=pg.mkPen('y', width=2))
        self.curve_act_z = self.plot_z.plot(pen=pg.mkPen('c', width=2))
        
        self.curve_err = self.plot_err.plot(pen=pg.mkPen('r', width=2))

        # Add legend
        self.plot_xy.addLegend()
        self.plot_xy.plot([0], [0], pen=pg.mkPen('y', width=2), name="Reference")
        self.plot_xy.plot([0], [0], pen=pg.mkPen('c', width=2), name="Actual")

        # Data buffers
        self.start_time = None
        self.history_t = []
        self.history_x = []
        self.history_y = []
        self.history_z = []
        
        self.ref_x = []
        self.ref_y = []
        self.ref_z = []
        self.err_history = []

        # ROS subscriber
        self.sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)

        # Qt Timer for GUI updates
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start(50)  # 20 Hz update

        self.get_logger().info('Live Plotter started (PyQtGraph)')

    def odom_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_time is None:
            self.start_time = t
            
        t_elapsed = t - self.start_time
        pos = msg.pose.pose.position
        
        # Standard ENU -> NED conversion for plotting
        act_x_ned = pos.y
        act_y_ned = pos.x
        act_z_ned = -pos.z

        ref = reference_trajectory(t_elapsed, shape=self.trajectory_shape, T_final=self.duration)
        rx = ref['eta_d'][0]
        ry = ref['eta_d'][1]
        rz = ref['eta_d'][2]
        
        err = np.sqrt((rx - act_x_ned)**2 + (ry - act_y_ned)**2 + (rz - act_z_ned)**2)
        
        self.history_t.append(t_elapsed)
        self.history_x.append(act_x_ned)
        self.history_y.append(act_y_ned)
        self.history_z.append(act_z_ned)
        
        self.ref_x.append(rx)
        self.ref_y.append(ry)
        self.ref_z.append(rz)
        self.err_history.append(err)
        
        # Keep last 1500 points (approx 30s at 50Hz)
        if len(self.history_t) > 1500:
            self.history_t.pop(0)
            self.history_x.pop(0)
            self.history_y.pop(0)
            self.history_z.pop(0)
            self.ref_x.pop(0)
            self.ref_y.pop(0)
            self.ref_z.pop(0)
            self.err_history.pop(0)

    def update_plot(self):
        if len(self.history_t) < 2:
            return
            
        # Update plot data
        self.curve_ref_xy.setData(self.ref_x, self.ref_y)
        self.curve_act_xy.setData(self.history_x, self.history_y)
        
        self.curve_ref_z.setData(self.history_t, self.ref_z)
        self.curve_act_z.setData(self.history_t, self.history_z)
        
        self.curve_err.setData(self.history_t, self.err_history)
        
        # Process Qt events to keep GUI responsive
        self.app.processEvents()

def main(args=None):
    rclpy.init(args=args)
    node = LivePlotter()
    
    if HAS_PG:
        try:
            # We must spin ROS manually inside the Qt event loop, or run spin in a thread
            # Simple approach: manually call spin_once in the timer
            node.update_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
            pg.exec()
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
