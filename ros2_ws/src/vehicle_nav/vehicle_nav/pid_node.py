#!/usr/bin/env python3
"""
PID Controller Node for trajectory tracking
Subscribes to /planned_path and /state
Publishes to /cmd_vel_nom
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import math

class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nom', 10)
        
        # Subscribers
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.create_subscription(PoseStamped, '/state', self.state_callback, 10)
        
        # Parameters
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('pid.kp_v', 1.0)
        self.declare_parameter('pid.ki_v', 0.1)
        self.declare_parameter('pid.kd_v', 0.05)
        self.declare_parameter('pid.kp_w', 2.0)
        self.declare_parameter('pid.ki_w', 0.2)
        self.declare_parameter('pid.kd_w', 0.1)
        self.declare_parameter('pid.lookahead_distance', 1.0)
        self.declare_parameter('vehicle.max_speed', 5.0)
        self.declare_parameter('vehicle.max_angular_velocity', math.pi / 2)
        
        # Get parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.kp_v = self.get_parameter('pid.kp_v').value
        self.ki_v = self.get_parameter('pid.ki_v').value
        self.kd_v = self.get_parameter('pid.kd_v').value
        self.kp_w = self.get_parameter('pid.kp_w').value
        self.ki_w = self.get_parameter('pid.ki_w').value
        self.kd_w = self.get_parameter('pid.kd_w').value
        self.lookahead_dist = self.get_parameter('pid.lookahead_distance').value
        self.v_max = self.get_parameter('vehicle.max_speed').value
        self.w_max = self.get_parameter('vehicle.max_angular_velocity').value
        
        # State variables
        self.current_state = None
        self.planned_path = None
        
        # PID state
        self.integral_v = 0.0
        self.integral_w = 0.0
        self.prev_error_v = 0.0
        self.prev_error_w = 0.0
        
        # Control timer
        self.dt = 1.0 / self.control_rate
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('PID Controller Node initialized')
    
    def path_callback(self, msg):
        """Store the planned path"""
        self.planned_path = msg
        # Reset PID state when new path is received
        self.reset()
        self.get_logger().info(f'Received path with {len(msg.poses)} waypoints')
    
    def state_callback(self, msg):
        """Store current state"""
        self.current_state = msg
    
    def reset(self):
        """Reset PID controller state"""
        self.integral_v = 0.0
        self.integral_w = 0.0
        self.prev_error_v = 0.0
        self.prev_error_w = 0.0
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def find_lookahead_point(self, current_pos, path_poses):
        """Find lookahead point on path using pure pursuit"""
        x, y = current_pos
        
        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose in enumerate(path_poses):
            px = pose.pose.position.x
            py = pose.pose.position.y
            dist = math.sqrt((px - x)**2 + (py - y)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Find lookahead point
        lookahead_idx = closest_idx
        for i in range(closest_idx, len(path_poses)):
            px = path_poses[i].pose.position.x
            py = path_poses[i].pose.position.y
            dist = math.sqrt((px - x)**2 + (py - y)**2)
            if dist >= self.lookahead_dist:
                lookahead_idx = i
                break
        
        # If at end of path, use last point
        if lookahead_idx >= len(path_poses) - 1:
            lookahead_idx = len(path_poses) - 1
        
        return lookahead_idx, min_dist
    
    def compute_control(self, current_state, desired_state):
        """
        Compute PID control command
        current_state: [x, y, theta]
        desired_state: [x, y, theta, v_desired]
        Returns: (v_cmd, w_cmd)
        """
        x, y, theta = current_state
        x_d, y_d, theta_d, v_d = desired_state
        
        # Transform error to local frame
        dx_global = x_d - x
        dy_global = y_d - y
        
        # Rotate to vehicle frame
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        
        e_x = cos_theta * dx_global + sin_theta * dy_global
        e_y = -sin_theta * dx_global + cos_theta * dy_global
        
        # Distance to goal
        dist = math.sqrt(e_x**2 + e_y**2)
        
        # Heading error (normalized to [-pi, pi])
        e_theta = self.normalize_angle(theta_d - theta)
        
        # Desired velocity based on distance
        if dist > 0.1:
            v_desired = min(v_d, dist)
        else:
            v_desired = 0.0
        
        # Velocity error (simplified - assuming current velocity is 0 for now)
        v_current = 0.0  # Could be estimated from state history if available
        error_v = v_desired - v_current
        
        # Update integral with anti-windup
        self.integral_v = self.integral_v + error_v * self.dt
        self.integral_w = self.integral_w + e_theta * self.dt
        
        # Anti-windup
        self.integral_v = max(min(self.integral_v, 1.0), -1.0)
        self.integral_w = max(min(self.integral_w, 1.0), -1.0)
        
        # Derivative
        deriv_v = (error_v - self.prev_error_v) / self.dt
        deriv_w = (e_theta - self.prev_error_w) / self.dt
        
        # PID control
        v_cmd = self.kp_v * error_v + self.ki_v * self.integral_v + self.kd_v * deriv_v
        w_cmd = self.kp_w * e_theta + self.ki_w * self.integral_w + self.kd_w * deriv_w
        
        # Add feedforward
        v_cmd = v_cmd + v_desired
        
        # Apply velocity limits
        v_cmd = max(min(v_cmd, self.v_max), 0.0)
        w_cmd = max(min(w_cmd, self.w_max), -self.w_max)
        
        # Store errors for next iteration
        self.prev_error_v = error_v
        self.prev_error_w = e_theta
        
        return v_cmd, w_cmd
    
    def control_loop(self):
        """Main control loop"""
        if self.current_state is None:
            self.get_logger().warn('No state received yet', throttle_duration_sec=2.0)
            return
            
        if self.planned_path is None:
            self.get_logger().warn('No path received yet', throttle_duration_sec=2.0)
            return
        
        # Check if path is empty
        if len(self.planned_path.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        
        # Extract current position and orientation
        x = self.current_state.pose.position.x
        y = self.current_state.pose.position.y
        theta = self.quaternion_to_yaw(self.current_state.pose.orientation)
        
        # Find lookahead point on path
        lookahead_idx, dist_to_path = self.find_lookahead_point((x, y), self.planned_path.poses)
        
        # Get desired state from lookahead point
        lookahead_pose = self.planned_path.poses[lookahead_idx]
        x_d = lookahead_pose.pose.position.x
        y_d = lookahead_pose.pose.position.y
        theta_d = self.quaternion_to_yaw(lookahead_pose.pose.orientation)
        v_d = 1.0  # desired velocity
        
        # Check if reached goal (last point in path)
        if lookahead_idx == len(self.planned_path.poses) - 1:
            goal_dist = math.sqrt((x_d - x)**2 + (y_d - y)**2)
            if goal_dist < 0.15:  # Goal tolerance
                # Reached goal, stop
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.get_logger().info('Goal reached!', throttle_duration_sec=2.0)
                return
        
        # Compute control command
        current = [x, y, theta]
        desired = [x_d, y_d, theta_d, v_d]
        v_cmd, w_cmd = self.compute_control(current, desired)
        
        # Log for debugging
        self.get_logger().info(
            f'Control: v={v_cmd:.2f}, w={w_cmd:.2f}, lookahead_idx={lookahead_idx}/{len(self.planned_path.poses)}',
            throttle_duration_sec=1.0
        )
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PIDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
