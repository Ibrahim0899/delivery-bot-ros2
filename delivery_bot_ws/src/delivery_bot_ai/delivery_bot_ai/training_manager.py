#!/usr/bin/env python3
"""
Training Manager for DeliveryBot
=================================
Manages the training loop for reinforcement learning navigation.
Handles episode management, metrics tracking, and training termination.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, Int32, Bool
from nav_msgs.msg import Odometry
import json
import os
from datetime import datetime
import math


class TrainingManager(Node):
    """
    ROS2 Node for managing Q-Learning training episodes.
    
    Subscribes:
        /training/episode_done - Signal that episode ended
        /training/episode_reward - Reward for current episode
        /odom - Robot odometry for goal detection
        
    Publishes:
        /training/episode_number - Current episode number
        /training/reset_signal - Signal to reset navigation
        
    Services:
        Calls /reset_robot to reset simulation
    """
    
    def __init__(self):
        super().__init__('training_manager')
        
        # Callback group for async service calls
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters
        self.declare_parameter('num_episodes', 500)
        self.declare_parameter('max_steps_per_episode', 1000)
        self.declare_parameter('success_threshold', 0.8)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 5.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('log_dir', '/tmp/delivery_bot_training')
        self.declare_parameter('auto_start', True)
        
        self.num_episodes = self.get_parameter('num_episodes').value
        self.max_steps = self.get_parameter('max_steps_per_episode').value
        self.success_threshold = self.get_parameter('success_threshold').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.log_dir = self.get_parameter('log_dir').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # State
        self.current_episode = 0
        self.episode_rewards = []
        self.episode_successes = []
        self.episode_steps = []
        self.current_reward = 0.0
        self.current_steps = 0
        self.training_active = False
        self.current_position = (0.0, 0.0)
        
        # Create log directory
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Publishers
        self.episode_pub = self.create_publisher(Int32, '/training/episode_number', 10)
        self.reset_signal_pub = self.create_publisher(Bool, '/training/reset_signal', 10)
        
        # Subscribers
        self.reward_sub = self.create_subscription(
            Float32,
            '/training/episode_reward',
            self.reward_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.done_sub = self.create_subscription(
            Bool,
            '/training/episode_done',
            self.episode_done_callback,
            10
        )
        
        # Service client for reset
        self.reset_client = self.create_client(
            Trigger,
            'reset_robot',
            callback_group=self.callback_group
        )
        
        # Training control timer
        self.control_timer = self.create_timer(0.1, self.training_control_loop)
        
        # Wait for reset service
        self.get_logger().info('Waiting for reset service...')
        
        if self.auto_start:
            self.create_timer(3.0, self.start_training, callback_group=self.callback_group)
    
    def start_training(self):
        """Start the training process."""
        if self.training_active:
            return
            
        self.training_active = True
        self.current_episode = 0
        self.get_logger().info(
            f'=== Starting Training: {self.num_episodes} episodes ==='
        )
        self.get_logger().info(
            f'Goal: ({self.goal_x}, {self.goal_y}), Tolerance: {self.goal_tolerance}m'
        )
        
        self.start_new_episode()
    
    def start_new_episode(self):
        """Initialize a new training episode."""
        self.current_episode += 1
        self.current_reward = 0.0
        self.current_steps = 0
        
        # Publish episode number
        msg = Int32()
        msg.data = self.current_episode
        self.episode_pub.publish(msg)
        
        # Reset robot position
        self.call_reset_service()
        
        self.get_logger().info(
            f'--- Episode {self.current_episode}/{self.num_episodes} started ---'
        )
    
    def call_reset_service(self):
        """Call the reset service asynchronously."""
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Reset service not available')
            return
        
        request = Trigger.Request()
        future = self.reset_client.call_async(request)
        future.add_done_callback(self.reset_done_callback)
    
    def reset_done_callback(self, future):
        """Handle reset service response."""
        try:
            response = future.result()
            if response.success:
                # Signal navigation agent to reset
                msg = Bool()
                msg.data = True
                self.reset_signal_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Reset call failed: {e}')
    
    def reward_callback(self, msg):
        """Accumulate episode reward."""
        self.current_reward += msg.data
        self.current_steps += 1
    
    def odom_callback(self, msg):
        """Track robot position."""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
    
    def episode_done_callback(self, msg):
        """Handle episode completion signal."""
        if msg.data and self.training_active:
            self.end_episode(success=self.check_goal_reached())
    
    def check_goal_reached(self):
        """Check if robot reached the goal."""
        dist = math.sqrt(
            (self.current_position[0] - self.goal_x) ** 2 +
            (self.current_position[1] - self.goal_y) ** 2
        )
        return dist <= self.goal_tolerance
    
    def training_control_loop(self):
        """Main training control loop."""
        if not self.training_active:
            return
        
        # Check for max steps
        if self.current_steps >= self.max_steps:
            self.end_episode(success=False)
    
    def end_episode(self, success: bool):
        """End current episode and record metrics."""
        self.episode_rewards.append(self.current_reward)
        self.episode_successes.append(success)
        self.episode_steps.append(self.current_steps)
        
        # Calculate running success rate
        recent_successes = self.episode_successes[-100:]
        success_rate = sum(recent_successes) / len(recent_successes)
        
        self.get_logger().info(
            f'Episode {self.current_episode}: '
            f'Reward={self.current_reward:.2f}, '
            f'Steps={self.current_steps}, '
            f'Success={success}, '
            f'Rate={success_rate:.2%}'
        )
        
        # Check termination conditions
        if self.current_episode >= self.num_episodes:
            self.finish_training('Max episodes reached')
        elif success_rate >= self.success_threshold and len(recent_successes) >= 50:
            self.finish_training(f'Success threshold {self.success_threshold:.0%} reached')
        else:
            self.start_new_episode()
    
    def finish_training(self, reason: str):
        """Complete training and save results."""
        self.training_active = False
        
        # Calculate final metrics
        total_successes = sum(self.episode_successes)
        avg_reward = sum(self.episode_rewards) / len(self.episode_rewards) if self.episode_rewards else 0
        final_success_rate = total_successes / len(self.episode_successes) if self.episode_successes else 0
        
        # Save training log
        log_data = {
            'timestamp': datetime.now().isoformat(),
            'reason': reason,
            'episodes': self.current_episode,
            'total_successes': total_successes,
            'success_rate': final_success_rate,
            'avg_reward': avg_reward,
            'episode_rewards': self.episode_rewards,
            'episode_successes': self.episode_successes,
            'episode_steps': self.episode_steps
        }
        
        log_file = os.path.join(
            self.log_dir, 
            f'training_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        )
        with open(log_file, 'w') as f:
            json.dump(log_data, f, indent=2)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'TRAINING COMPLETE: {reason}')
        self.get_logger().info(f'Episodes: {self.current_episode}')
        self.get_logger().info(f'Success Rate: {final_success_rate:.2%}')
        self.get_logger().info(f'Avg Reward: {avg_reward:.2f}')
        self.get_logger().info(f'Log saved: {log_file}')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = TrainingManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
