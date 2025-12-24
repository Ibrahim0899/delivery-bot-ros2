#!/usr/bin/env python3
"""
Training Launch File for DeliveryBot
=====================================
Launches the complete training environment including:
- Reset service for robot teleportation
- Training manager for episode control
- Navigation agent in training mode
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    num_episodes = LaunchConfiguration('num_episodes')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    
    declare_num_episodes = DeclareLaunchArgument(
        'num_episodes',
        default_value='500',
        description='Number of training episodes'
    )
    
    declare_start_x = DeclareLaunchArgument(
        'start_x',
        default_value='0.0',
        description='Start X coordinate (Point A)'
    )
    
    declare_start_y = DeclareLaunchArgument(
        'start_y',
        default_value='0.0',
        description='Start Y coordinate (Point A)'
    )
    
    declare_goal_x = DeclareLaunchArgument(
        'goal_x',
        default_value='5.0',
        description='Goal X coordinate (Point B)'
    )
    
    declare_goal_y = DeclareLaunchArgument(
        'goal_y',
        default_value='5.0',
        description='Goal Y coordinate (Point B)'
    )
    
    # Reset Service Node
    reset_service = Node(
        package='delivery_bot_ai',
        executable='reset_service',
        name='reset_service',
        output='screen',
        parameters=[{
            'start_x': start_x,
            'start_y': start_y,
            'start_z': 0.2,
            'world_name': 'delivery_zone',
            'robot_name': 'delivery_bot',
            'use_sim_time': True
        }]
    )
    
    # Training Manager Node
    training_manager = Node(
        package='delivery_bot_ai',
        executable='training_manager',
        name='training_manager',
        output='screen',
        parameters=[{
            'num_episodes': num_episodes,
            'max_steps_per_episode': 1000,
            'success_threshold': 0.8,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_tolerance': 0.5,
            'log_dir': '/tmp/delivery_bot_training',
            'auto_start': True,
            'use_sim_time': True
        }]
    )
    
    # Navigation Agent in Training Mode
    navigation_agent = Node(
        package='delivery_bot_ai',
        executable='obstacle_avoidance_agent',
        name='obstacle_avoidance_agent',
        output='screen',
        parameters=[{
            'start_x': start_x,
            'start_y': start_y,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_tolerance': 0.5,
            'training_mode': True,
            'q_table_path': '/tmp/delivery_bot_qtable.pkl',
            'control_rate': 10.0,
            'use_sim_time': True
        }]
    )
    
    return LaunchDescription([
        declare_num_episodes,
        declare_start_x,
        declare_start_y,
        declare_goal_x,
        declare_goal_y,
        reset_service,
        training_manager,
        navigation_agent,
    ])
