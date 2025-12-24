#!/usr/bin/env python3
"""
Launch file for the DeliveryBot AI navigation agent.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    training_mode = LaunchConfiguration('training_mode')
    
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
    
    declare_training_mode = DeclareLaunchArgument(
        'training_mode',
        default_value='true',
        description='Enable training mode with exploration'
    )
    
    # AI Navigation Agent Node
    obstacle_avoidance_agent = Node(
        package='delivery_bot_ai',
        executable='obstacle_avoidance_agent',
        name='obstacle_avoidance_agent',
        output='screen',
        parameters=[{
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_tolerance': 0.5,
            'training_mode': training_mode,
            'q_table_path': '/tmp/delivery_bot_qtable.pkl',
            'control_rate': 10.0,
            'use_sim_time': True
        }]
    )
    
    return LaunchDescription([
        declare_goal_x,
        declare_goal_y,
        declare_training_mode,
        obstacle_avoidance_agent
    ])
