#!/usr/bin/env python3
"""
Reset Service for DeliveryBot
==============================
Provides a ROS2 service to reset the robot position in Gazebo simulation.
Uses Gazebo transport to teleport the robot to specified coordinates.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
import subprocess
import time


class ResetService(Node):
    """
    ROS2 service node to reset robot position in Gazebo.
    
    Services:
        /reset_robot - Resets robot to start position
    """
    
    def __init__(self):
        super().__init__('reset_service')
        
        # Parameters for start position
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_z', 0.2)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('world_name', 'delivery_zone')
        self.declare_parameter('robot_name', 'delivery_bot')
        
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_z = self.get_parameter('start_z').value
        self.start_yaw = self.get_parameter('start_yaw').value
        self.world_name = self.get_parameter('world_name').value
        self.robot_name = self.get_parameter('robot_name').value
        
        # Publisher to stop the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create reset service
        self.reset_srv = self.create_service(
            Trigger,
            'reset_robot',
            self.reset_callback
        )
        
        self.get_logger().info(
            f'Reset service ready. Start position: ({self.start_x}, {self.start_y})'
        )
    
    def reset_callback(self, request, response):
        """Handle reset request."""
        try:
            # First, stop the robot
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.1)
            
            # Use gz service to set pose
            # Format: gz service -s /world/{world}/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 1000 --req 'name: "{robot}", position: {x: X, y: Y, z: Z}'
            pose_cmd = (
                f'gz service -s /world/{self.world_name}/set_pose '
                f'--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 '
                f'--req \'name: "{self.robot_name}", '
                f'position: {{x: {self.start_x}, y: {self.start_y}, z: {self.start_z}}}, '
                f'orientation: {{x: 0, y: 0, z: 0, w: 1}}\''
            )
            
            result = subprocess.run(
                pose_cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                # Stop robot again after teleport
                self.cmd_vel_pub.publish(stop_msg)
                
                response.success = True
                response.message = f'Robot reset to ({self.start_x}, {self.start_y})'
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f'Reset failed: {result.stderr}'
                self.get_logger().error(response.message)
                
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = 'Reset timed out'
            self.get_logger().error(response.message)
        except Exception as e:
            response.success = False
            response.message = f'Reset error: {str(e)}'
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
