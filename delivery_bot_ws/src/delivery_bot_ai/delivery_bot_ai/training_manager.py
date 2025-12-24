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
from std_msgs.msg import Float32, Int32, Bool, String
from nav_msgs.msg import Odometry
import json
import os
from datetime import datetime
import math

# Try to import matplotlib for plotting
try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


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
        /training/save_model - Signal to save Q-table
        
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
        
        # Best model tracking
        self.best_success_rate = 0.0
        self.best_models = []  # List of (success_rate, timestamp, episode)
        
        # Create log directory
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Publishers
        self.episode_pub = self.create_publisher(Int32, '/training/episode_number', 10)
        self.reset_signal_pub = self.create_publisher(Bool, '/training/reset_signal', 10)
        self.save_model_pub = self.create_publisher(String, '/training/save_model', 10)
        
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
        
        # Signal agent to save Q-table every episode
        save_msg = String()
        save_msg.data = f'{self.log_dir}/qtable_ep{self.current_episode}.pkl'
        self.save_model_pub.publish(save_msg)
        
        # Check if this is a new best model (after 10+ episodes)
        if len(recent_successes) >= 10 and success_rate > self.best_success_rate:
            self.best_success_rate = success_rate
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.best_models.append((success_rate, timestamp, self.current_episode))
            
            # Keep only top 3 models
            self.best_models.sort(key=lambda x: x[0], reverse=True)
            self.best_models = self.best_models[:3]
            
            # Signal to save best model
            best_msg = String()
            best_msg.data = f'{self.log_dir}/best_qtable_{timestamp}_ep{self.current_episode}.pkl'
            self.save_model_pub.publish(best_msg)
            self.get_logger().info(f'🏆 New best model! Rate: {success_rate:.2%}')
        
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
            'best_models': [(m[0], m[1], m[2]) for m in self.best_models],
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
        
        # Generate plots
        self.plot_metrics()
        
        # Signal final save
        final_save = String()
        final_save.data = f'{self.log_dir}/final_qtable.pkl'
        self.save_model_pub.publish(final_save)
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'TRAINING COMPLETE: {reason}')
        self.get_logger().info(f'Episodes: {self.current_episode}')
        self.get_logger().info(f'Success Rate: {final_success_rate:.2%}')
        self.get_logger().info(f'Avg Reward: {avg_reward:.2f}')
        self.get_logger().info(f'Best Models: {len(self.best_models)}')
        self.get_logger().info(f'Log saved: {log_file}')
        self.get_logger().info('=' * 50)
    
    def plot_metrics(self):
        """Generate and save training metrics plots."""
        if not MATPLOTLIB_AVAILABLE:
            self.get_logger().warn('matplotlib not available, skipping plots')
            return
        
        if len(self.episode_rewards) < 2:
            return
            
        try:
            fig, axes = plt.subplots(2, 2, figsize=(12, 10))
            fig.suptitle('Training Metrics', fontsize=14, fontweight='bold')
            
            episodes = list(range(1, len(self.episode_rewards) + 1))
            
            # Plot 1: Rewards per episode
            ax1 = axes[0, 0]
            ax1.plot(episodes, self.episode_rewards, 'b-', alpha=0.5, label='Reward')
            # Moving average
            window = min(20, len(self.episode_rewards))
            if window > 1:
                ma = [sum(self.episode_rewards[max(0,i-window):i])/min(i,window) 
                      for i in range(1, len(self.episode_rewards)+1)]
                ax1.plot(episodes, ma, 'r-', linewidth=2, label=f'MA({window})')
            ax1.set_xlabel('Episode')
            ax1.set_ylabel('Reward')
            ax1.set_title('Episode Rewards')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # Plot 2: Cumulative success rate
            ax2 = axes[0, 1]
            cumsum = [sum(self.episode_successes[:i+1])/(i+1) * 100 
                      for i in range(len(self.episode_successes))]
            ax2.plot(episodes, cumsum, 'g-', linewidth=2)
            ax2.axhline(y=self.success_threshold * 100, color='r', linestyle='--', 
                       label=f'Target ({self.success_threshold:.0%})')
            ax2.set_xlabel('Episode')
            ax2.set_ylabel('Success Rate (%)')
            ax2.set_title('Cumulative Success Rate')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
            ax2.set_ylim(0, 100)
            
            # Plot 3: Steps per episode
            ax3 = axes[1, 0]
            ax3.plot(episodes, self.episode_steps, 'm-', alpha=0.7)
            ax3.axhline(y=self.max_steps, color='r', linestyle='--', label='Max steps')
            ax3.set_xlabel('Episode')
            ax3.set_ylabel('Steps')
            ax3.set_title('Steps per Episode')
            ax3.legend()
            ax3.grid(True, alpha=0.3)
            
            # Plot 4: Success/Fail distribution
            ax4 = axes[1, 1]
            success_count = sum(self.episode_successes)
            fail_count = len(self.episode_successes) - success_count
            ax4.bar(['Success', 'Fail'], [success_count, fail_count], 
                   color=['green', 'red'], alpha=0.7)
            ax4.set_ylabel('Count')
            ax4.set_title(f'Results: {success_count}/{len(self.episode_successes)} Success')
            
            plt.tight_layout()
            
            plot_file = os.path.join(self.log_dir, 'training_metrics.png')
            plt.savefig(plot_file, dpi=150, bbox_inches='tight')
            plt.close()
            
            self.get_logger().info(f'📊 Metrics plot saved: {plot_file}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to generate plots: {e}')


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
