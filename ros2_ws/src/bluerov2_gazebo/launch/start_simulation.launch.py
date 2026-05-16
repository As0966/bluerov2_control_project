"""
BlueROV2 Full Simulation — Start Gazebo Harmonic + Spawn Robot + Bridge Topics

This is the **main one-command launch** for the BlueROV2 simulation.

Usage:
  ros2 launch bluerov2_gazebo start_simulation.launch.py
  ros2 launch bluerov2_gazebo start_simulation.launch.py x:=2.0 depth:=-90.0
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('bluerov2_gazebo')
    desc_pkg   = get_package_share_directory('bluerov2_description')

    default_world = os.path.join(gazebo_pkg, 'worlds', 'underwater_world.sdf')

    gz_resource = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(desc_pkg, '..'),
    )

    # Fix FastDDS "sequence size exceeds remaining buffer" issue
    fastdds_fix = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(os.path.expanduser('~'),
                     'bluerov2_control_project', 'fastdds_profile.xml'),
    )

    bridge_args = [
        # Odometry
        '/model/bluerov2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        # IMU
        '/model/bluerov2/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        # Clock
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        # Body wrench (ROS2 → Gazebo) for direct force application
        '/model/bluerov2/body_wrench@geometry_msgs/msg/Wrench]gz.msgs.Wrench',
    ]

    # ── Remappings: sensor topics ────────────────────────────────
    remappings = [
        ('/model/bluerov2/odometry',    '/odom'),
        ('/model/bluerov2/imu',         '/imu'),
    ]

    return LaunchDescription([
        gz_resource,
        fastdds_fix,

        # ── Arguments ─────────────────────────────────────────────
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Full path to the SDF world file'),
        DeclareLaunchArgument('x',     default_value='0.0'),
        DeclareLaunchArgument('y',     default_value='0.0'),
        DeclareLaunchArgument('depth', default_value='3.0',
                              description='Z position (positive = above ground)'),
        DeclareLaunchArgument('yaw',   default_value='0.0'),

        DeclareLaunchArgument('gui', default_value='true',
                              description='Launch Gazebo GUI (set false for headless)'),

        # ── 1. Start Gazebo Harmonic ──────────────────────────────
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v4', LaunchConfiguration('world')],
            output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': os.path.join(desc_pkg, '..'),
            },
        ),

        # ── 2. Spawn BlueROV2 ────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'spawn_bluerov2.launch.py')
            ),
            launch_arguments={
                'x':     LaunchConfiguration('x'),
                'y':     LaunchConfiguration('y'),
                'depth': LaunchConfiguration('depth'),
                'yaw':   LaunchConfiguration('yaw'),
            }.items(),
        ),

        # ── 3. ros_gz_bridge — Gazebo ↔ ROS 2 topic bridge ──────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            arguments=bridge_args,
            remappings=remappings,
        ),
    ])
