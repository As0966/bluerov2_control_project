"""
BlueROV2 Empty Simulation — Lightweight world for controller development.

Usage:
  ros2 launch bluerov2_gazebo start_empty_sim.launch.py
  ros2 launch bluerov2_gazebo start_empty_sim.launch.py depth:=-50.0
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

    world_file = os.path.join(gazebo_pkg, 'worlds', 'empty_underwater.sdf')

    # ── Bridge args ──────────────────────────────────────────────
    bridge_args = [
        '/model/bluerov2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/model/bluerov2/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/model/bluerov2/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    ]
    for i in range(6):
        bridge_args.append(
            f'/model/bluerov2/joint/thruster{i}_joint/cmd_thrust'
            f'@std_msgs/msg/Float64]gz.msgs.Double'
        )

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(desc_pkg, '..'),
        ),

        DeclareLaunchArgument('x',     default_value='0.0'),
        DeclareLaunchArgument('y',     default_value='0.0'),
        DeclareLaunchArgument('depth', default_value='1.0'),
        DeclareLaunchArgument('yaw',   default_value='0.0'),

        # ── 1. Start Gazebo with empty underwater world ──────────
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
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

        # ── 3. ros_gz_bridge ─────────────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            arguments=bridge_args,
            remappings=[
                ('/model/bluerov2/odometry',    '/odom'),
                ('/model/bluerov2/imu',         '/imu'),
                ('/model/bluerov2/joint_states', '/joint_states'),
            ],
        ),
    ])
