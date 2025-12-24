#!/usr/bin/env python3
"""
Obstacle Avoidance Agent for DeliveryBot
=========================================
A Q-Learning based navigation agent that navigates from Point A (0,0) 
to Point B (5,5) while avoiding obstacles.

Features:
- Discretized state space from LiDAR readings
- Q-Learning with epsilon-greedy exploration
- Reward shaping for goal-seeking and collision avoidance
- Persistent Q-table storage
"""

import math
import pickle
import os
from typing import Tuple, List
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String


class QLearningAgent:
    """
    Q-Learning agent for discrete state-action navigation.
    
    State Space: Discretized LiDAR sectors (front, front-left, front-right, left, right)
                 Each sector: CLEAR (0), WARN (1), DANGER (2)
                 Plus: goal direction (8 directions)
                 
    Action Space: 
        0 - Forward
        1 - Turn Left
        2 - Turn Right
        3 - Forward-Left
        4 - Forward-Right
        5 - Stop
    """
    
    def __init__(
        self,
        learning_rate: float = 0.1,
        discount_factor: float = 0.95,
        epsilon: float = 0.3,
        epsilon_decay: float = 0.995,
        epsilon_min: float = 0.05
    ):
        self.alpha = learning_rate
        self.gamma = discount_factor
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min
        
        # Q-table as a defaultdict for sparse storage
        self.q_table = defaultdict(lambda: np.zeros(6))
        
        # Action definitions: (linear_x, angular_z)
        self.actions = {
            0: (0.3, 0.0),    # Forward
            1: (0.0, 0.5),    # Turn Left
            2: (0.0, -0.5),   # Turn Right
            3: (0.2, 0.3),    # Forward-Left
            4: (0.2, -0.3),   # Forward-Right
            5: (0.0, 0.0),    # Stop
        }
        
        # Distance thresholds for discretization (meters)
        self.DANGER_DIST = 0.4
        self.WARN_DIST = 1.0
        
    def discretize_lidar(self, ranges: List[float], num_sectors: int = 5) -> Tuple[int, ...]:
        """
        Discretize LiDAR readings into sectors.
        
        Args:
            ranges: Raw LiDAR range data (360 degrees)
            num_sectors: Number of sectors to divide the scan
            
        Returns:
            Tuple of discretized sector values
        """
        if not ranges or len(ranges) == 0:
            return tuple([2] * num_sectors)  # Assume danger if no data
            
        n = len(ranges)
        sector_size = n // num_sectors
        sectors = []
        
        for i in range(num_sectors):
            start_idx = i * sector_size
            end_idx = start_idx + sector_size
            sector_ranges = ranges[start_idx:end_idx]
            
            # Filter out inf/nan values
            valid_ranges = [r for r in sector_ranges if not math.isinf(r) and not math.isnan(r)]
            
            if not valid_ranges:
                sectors.append(0)  # Clear if no valid readings
            else:
                min_range = min(valid_ranges)
                if min_range < self.DANGER_DIST:
                    sectors.append(2)  # DANGER
                elif min_range < self.WARN_DIST:
                    sectors.append(1)  # WARN
                else:
                    sectors.append(0)  # CLEAR
                    
        return tuple(sectors)
    
    def get_goal_direction(self, current_pos: Tuple[float, float], 
                           current_yaw: float,
                           goal: Tuple[float, float] = (5.0, 5.0)) -> int:
        """
        Get discretized goal direction (8 directions).
        
        Returns:
            Direction index 0-7 (N, NE, E, SE, S, SW, W, NW relative to robot)
        """
        dx = goal[0] - current_pos[0]
        dy = goal[1] - current_pos[1]
        
        # Angle to goal in world frame
        angle_to_goal = math.atan2(dy, dx)
        
        # Relative angle (goal direction in robot frame)
        relative_angle = angle_to_goal - current_yaw
        
        # Normalize to [-pi, pi]
        while relative_angle > math.pi:
            relative_angle -= 2 * math.pi
        while relative_angle < -math.pi:
            relative_angle += 2 * math.pi
            
        # Discretize into 8 directions
        direction = int((relative_angle + math.pi) / (math.pi / 4)) % 8
        return direction
    
    def get_state(self, lidar_ranges: List[float], 
                  position: Tuple[float, float],
                  yaw: float) -> Tuple:
        """
        Create full state representation.
        """
        lidar_state = self.discretize_lidar(lidar_ranges)
        goal_dir = self.get_goal_direction(position, yaw)
        return lidar_state + (goal_dir,)
    
    def choose_action(self, state: Tuple, training: bool = True) -> int:
        """
        Epsilon-greedy action selection.
        """
        if training and np.random.random() < self.epsilon:
            return np.random.randint(0, len(self.actions))
        return int(np.argmax(self.q_table[state]))
    
    def get_velocity(self, action: int) -> Tuple[float, float]:
        """
        Get velocity command for action.
        """
        return self.actions[action]
    
    def update(self, state: Tuple, action: int, reward: float, next_state: Tuple):
        """
        Q-Learning update rule.
        """
        current_q = self.q_table[state][action]
        max_next_q = np.max(self.q_table[next_state])
        new_q = current_q + self.alpha * (reward + self.gamma * max_next_q - current_q)
        self.q_table[state][action] = new_q
        
    def decay_epsilon(self):
        """
        Decay exploration rate.
        """
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
        
    def save(self, filepath: str):
        """
        Save Q-table to file.
        """
        with open(filepath, 'wb') as f:
            pickle.dump(dict(self.q_table), f)
            
    def load(self, filepath: str):
        """
        Load Q-table from file.
        """
        if os.path.exists(filepath):
            with open(filepath, 'rb') as f:
                loaded = pickle.load(f)
                self.q_table = defaultdict(lambda: np.zeros(6), loaded)


class ObstacleAvoidanceAgent(Node):
    """
    ROS 2 Node for Q-Learning based obstacle avoidance navigation.
    
    Subscribes:
        /scan - LaserScan from LiDAR
        /odom - Odometry for position tracking
        
    Publishes:
        /cmd_vel - Velocity commands
    """
    
    def __init__(self):
        super().__init__('obstacle_avoidance_agent')
        
        # Declare parameters
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 5.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('training_mode', True)
        self.declare_parameter('q_table_path', '/tmp/delivery_bot_qtable.pkl')
        self.declare_parameter('control_rate', 10.0)
        
        # Get parameters
        self.goal = (
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value
        )
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.training_mode = self.get_parameter('training_mode').value
        self.q_table_path = self.get_parameter('q_table_path').value
        control_rate = self.get_parameter('control_rate').value
        
        # Initialize Q-Learning agent
        self.agent = QLearningAgent()
        self.agent.load(self.q_table_path)
        
        # State variables
        self.current_scan = None
        self.current_position = (0.0, 0.0)
        self.current_yaw = 0.0
        self.previous_state = None
        self.previous_action = None
        self.previous_distance = None
        
        # Statistics
        self.episode_reward = 0.0
        self.step_count = 0
        self.collision_count = 0
        self.goal_reached_count = 0
        
        # QoS for sensor data - must match Gazebo bridge (RELIABLE)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reward_pub = self.create_publisher(Float32, '/training/episode_reward', 10)
        self.done_pub = self.create_publisher(Bool, '/training/episode_done', 10)
        
        # Subscribe to reset signal from training manager
        self.reset_sub = self.create_subscription(
            Bool,
            '/training/reset_signal',
            self.reset_callback,
            10
        )
        
        # Subscribe to save_model signal from training manager
        self.save_model_sub = self.create_subscription(
            String,
            '/training/save_model',
            self.save_model_callback,
            10
        )
        
        # Control timer
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        
        self.get_logger().info('='*50)
        self.get_logger().info('DeliveryBot Obstacle Avoidance Agent Started')
        self.get_logger().info(f'Goal: ({self.goal[0]}, {self.goal[1]})')
        self.get_logger().info(f'Training Mode: {self.training_mode}')
        self.get_logger().info('='*50)
        
    def scan_callback(self, msg: LaserScan):
        """Process incoming LiDAR scan."""
        self.current_scan = list(msg.ranges)
        
    def odom_callback(self, msg: Odometry):
        """Process incoming odometry."""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def reset_callback(self, msg: Bool):
        """Handle reset signal from training manager."""
        if msg.data:
            self.get_logger().info('Resetting episode state...')
            # Reset episode state
            self.previous_state = None
            self.previous_action = None
            self.previous_distance = None
            self.episode_reward = 0.0
            self.step_count = 0
            # Decay epsilon at end of episode
            if self.training_mode:
                self.agent.decay_epsilon()
    
    def signal_episode_done(self, success: bool):
        """Signal episode completion to training manager."""
        done_msg = Bool()
        done_msg.data = True
        self.done_pub.publish(done_msg)
        
        self.get_logger().info(
            f'Episode ended: {"SUCCESS" if success else "FAILED"} | '
            f'Reward: {self.episode_reward:.2f}'
        )
    
    def save_model_callback(self, msg: String):
        """Save Q-table to specified path when signaled by training manager."""
        save_path = msg.data
        if save_path and self.training_mode:
            try:
                self.agent.save(save_path)
                self.get_logger().info(
                    f'💾 Q-table saved: {save_path} ({len(self.agent.q_table)} states)'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to save Q-table: {e}')
        
    def calculate_reward(self) -> float:
        """
        Calculate reward based on current state.
        
        Reward Components:
        - Goal reached: +100
        - Collision (obstacle too close): -50
        - Getting closer to goal: +1 to +5 based on distance improvement
        - Moving away from goal: -1 to -5
        - Time penalty: -0.1 per step
        """
        reward = 0.0
        
        # Calculate distance to goal
        distance_to_goal = math.sqrt(
            (self.goal[0] - self.current_position[0])**2 +
            (self.goal[1] - self.current_position[1])**2
        )
        
        # Check for goal reached
        if distance_to_goal < self.goal_tolerance:
            self.get_logger().info('🎉 GOAL REACHED!')
            self.goal_reached_count += 1
            self.signal_episode_done(success=True)
            return 100.0
        
        # Check for collision (any obstacle too close)
        if self.current_scan:
            valid_ranges = [r for r in self.current_scan 
                          if not math.isinf(r) and not math.isnan(r)]
            if valid_ranges:
                min_distance = min(valid_ranges)
                if min_distance < 0.25:
                    self.get_logger().warn('⚠️ COLLISION DETECTED!')
                    self.collision_count += 1
                    self.signal_episode_done(success=False)
                    return -50.0
                # Penalty for being close to obstacles
                if min_distance < 0.5:
                    reward -= (0.5 - min_distance) * 10
        
        # Reward for getting closer to goal
        if self.previous_distance is not None:
            distance_improvement = self.previous_distance - distance_to_goal
            reward += distance_improvement * 10  # Scale factor
            
        self.previous_distance = distance_to_goal
        
        # Small time penalty to encourage efficiency
        reward -= 0.1
        
        # Publish step reward for training manager
        reward_msg = Float32()
        reward_msg.data = reward
        self.reward_pub.publish(reward_msg)
        
        return reward
    
    def control_loop(self):
        """
        Main control loop - runs at control_rate Hz.
        """
        if self.current_scan is None:
            return
            
        self.step_count += 1
        
        # Get current state
        current_state = self.agent.get_state(
            self.current_scan,
            self.current_position,
            self.current_yaw
        )
        
        # Calculate reward for previous action
        if self.previous_state is not None and self.training_mode:
            reward = self.calculate_reward()
            self.episode_reward += reward
            self.agent.update(
                self.previous_state,
                self.previous_action,
                reward,
                current_state
            )
        
        # Choose action
        action = self.agent.choose_action(current_state, self.training_mode)
        linear_x, angular_z = self.agent.get_velocity(action)
        
        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)
        
        # Store for next iteration
        self.previous_state = current_state
        self.previous_action = action
        
        # Periodic logging
        if self.step_count % 50 == 0:
            distance = math.sqrt(
                (self.goal[0] - self.current_position[0])**2 +
                (self.goal[1] - self.current_position[1])**2
            )
            self.get_logger().info(
                f'Step {self.step_count} | '
                f'Pos: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}) | '
                f'Dist: {distance:.2f}m | '
                f'Reward: {self.episode_reward:.1f} | '
                f'ε: {self.agent.epsilon:.3f}'
            )
            
        # Decay epsilon periodically
        if self.training_mode and self.step_count % 100 == 0:
            self.agent.decay_epsilon()
            
        # Save Q-table periodically
        if self.training_mode and self.step_count % 500 == 0:
            self.agent.save(self.q_table_path)
            self.get_logger().info(f'Q-table saved ({len(self.agent.q_table)} states)')
            
    def destroy_node(self):
        """Cleanup on shutdown."""
        # Stop the robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # Save Q-table
        if self.training_mode:
            self.agent.save(self.q_table_path)
            self.get_logger().info('Final Q-table saved')
            
        # Print statistics
        self.get_logger().info('='*50)
        self.get_logger().info('Session Statistics:')
        self.get_logger().info(f'  Total Steps: {self.step_count}')
        self.get_logger().info(f'  Total Reward: {self.episode_reward:.2f}')
        self.get_logger().info(f'  Collisions: {self.collision_count}')
        self.get_logger().info(f'  Goals Reached: {self.goal_reached_count}')
        self.get_logger().info(f'  Q-table Size: {len(self.agent.q_table)} states')
        self.get_logger().info('='*50)
        
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    agent_node = ObstacleAvoidanceAgent()
    
    try:
        rclpy.spin(agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        agent_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
