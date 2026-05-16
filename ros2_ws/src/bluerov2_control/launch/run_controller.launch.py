"""
Launch a BlueROV2 controller node.

Usage:
  ros2 launch bluerov2_control run_controller.launch.py
  ros2 launch bluerov2_control run_controller.launch.py controller:=state_feedback
Available controllers:
  backstepping, state_feedback, feedback_linearization,
  observer, observer_linear, mrac
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller',
            default_value='backstepping',
            description='Controller to use: backstepping, state_feedback, '
                        'feedback_linearization, observer, '
                        'observer_linear, mrac, inn'),
        
        DeclareLaunchArgument(
            'duration',
            default_value='80.0',
            description='Simulation duration in seconds before auto-shutdown'),
            
        DeclareLaunchArgument(
            'disturbance',
            default_value='false',
            description='Whether to enable external disturbances (true/false)'),
            
        DeclareLaunchArgument(
            'trajectory',
            default_value='spline',
            description='Trajectory shape: spline, circle, figure8'),

        Node(
            package='bluerov2_control',
            executable='controller_node',
            name='bluerov2_controller',
            output='screen',
            parameters=[{
                'controller': LaunchConfiguration('controller'),
                'duration': LaunchConfiguration('duration'),
                'disturbance': LaunchConfiguration('disturbance'),
                'trajectory': LaunchConfiguration('trajectory'),
                'use_sim_time': True,
            }],
            on_exit=Shutdown(),
        ),

        # Wrench forwarder — applies body wrench to Gazebo
        Node(
            package='bluerov2_control',
            executable='wrench_forwarder',
            name='wrench_forwarder',
            output='screen',
        ),

        # Live Plotter — visualizations for Gazebo telemetry
        Node(
            package='bluerov2_control',
            executable='live_plotter',
            name='live_plotter',
            output='screen',
            parameters=[{
                'trajectory': LaunchConfiguration('trajectory'),
                'duration': LaunchConfiguration('duration'),
                'use_sim_time': True,
            }],
        ),
    ])
