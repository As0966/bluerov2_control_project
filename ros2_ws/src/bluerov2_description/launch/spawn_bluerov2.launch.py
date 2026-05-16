"""
BlueROV2 Description — Spawn the robot into a running Gazebo Harmonic instance.

Usage:
  ros2 launch bluerov2_description spawn_bluerov2.launch.py
  ros2 launch bluerov2_description spawn_bluerov2.launch.py x:=-5.0 depth:=-90.0 yaw:=1.57
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('bluerov2_description')
    sdf_file = os.path.join(pkg_dir, 'urdf', 'bluerov2.sdf')

    # Read the SDF so robot_state_publisher can serve it
    with open(sdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # ── Launch arguments ──────────────────────────────────────
        DeclareLaunchArgument('x',     default_value='0.0',
                              description='Initial X position (m)'),
        DeclareLaunchArgument('y',     default_value='0.0',
                              description='Initial Y position (m)'),
        DeclareLaunchArgument('depth', default_value='1.0',
                              description='Initial Z position — negative is deeper (m)'),
        DeclareLaunchArgument('yaw',   default_value='0.0',
                              description='Initial heading (rad)'),

        # ── robot_state_publisher ─────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
            }],
        ),

        # ── Spawn into Gazebo via ros_gz_sim ──────────────────────
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_bluerov2',
            output='screen',
            arguments=[
                '-file',  sdf_file,
                '-name',  'bluerov2',
                '-x',     LaunchConfiguration('x'),
                '-y',     LaunchConfiguration('y'),
                '-z',     LaunchConfiguration('depth'),
                '-Y',     LaunchConfiguration('yaw'),
            ],
        ),
    ])
