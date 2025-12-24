#!/usr/bin/env python3
"""
Main simulation launch file for DeliveryBot.
Launches Gazebo Harmonic, spawns the robot, starts robot_state_publisher,
ros_gz_bridge, and RViz.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_gazebo = get_package_share_directory('delivery_bot_gazebo')
    pkg_description = get_package_share_directory('delivery_bot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # File paths
    world_file = os.path.join(pkg_gazebo, 'worlds', 'delivery_zone.sdf')
    urdf_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_description, 'config', 'display.rviz')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Set Gazebo resource path
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_gazebo, 'worlds')]
    )
    
    # Robot description from xacro
    robot_description = Command(['xacro ', urdf_file])
    
    # ==================== GAZEBO ====================
    # Launch Gazebo Harmonic with the world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # ==================== SPAWN ROBOT ====================
    # Spawn the robot in Gazebo at Point A (0, 0)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'delivery_bot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # ==================== ROBOT STATE PUBLISHER ====================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # ==================== ROS-GZ BRIDGE ====================
    # Bridge topics between Gazebo and ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            # Velocity command (ROS -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Odometry (Gazebo -> ROS)
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # LiDAR scan (Gazebo -> ROS)
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            # TF (Gazebo -> ROS)
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            # Joint states (Gazebo -> ROS)
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            # Clock (Gazebo -> ROS) - Must use world-specific topic
            '/world/delivery_zone/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Camera (Gazebo -> ROS)
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/world/delivery_zone/clock', '/clock'),
        ]
    )
    
    # ==================== RVIZ ====================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz,
    ])
