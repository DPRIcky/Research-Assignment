#!/usr/bin/env python3
"""
Simple Goal Publisher Node
Publishes goal positions for testing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from math import sin, cos
import sys


class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        # Publisher
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)
        
        self.get_logger().info('Goal Publisher Node initialized')
        self.get_logger().info('Usage: ros2 run vehicle_nav goal_publisher <x> <y> [theta]')
    
    def publish_goal(self, x, y, theta=0.0):
        """Publish a goal position"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        
        # Convert theta to quaternion
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = sin(theta / 2.0)
        msg.pose.orientation.w = cos(theta / 2.0)
        
        # Publish with slight delay to ensure subscribers are ready
        rclpy.spin_once(self, timeout_sec=0.5)
        self.goal_pub.publish(msg)
        
        self.get_logger().info(f'Published goal: x={x}, y={y}, theta={theta}')


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 3:
        print('Usage: ros2 run vehicle_nav goal_publisher <x> <y> [theta]')
        print('Example: ros2 run vehicle_nav goal_publisher 10.0 10.0 0.0')
        return
    
    node = GoalPublisher()
    
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        
        node.publish_goal(x, y, theta)
        
        # Spin briefly to ensure message is sent
        rclpy.spin_once(node, timeout_sec=1.0)
        
    except ValueError:
        node.get_logger().error('Invalid arguments. Use: <x> <y> [theta]')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
