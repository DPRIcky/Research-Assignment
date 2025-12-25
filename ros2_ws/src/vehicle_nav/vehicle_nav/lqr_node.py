#!/usr/bin/env python3
"""
LQR Controller Node for trajectory tracking
Subscribes to /planned_path and /state
Publishes to /cmd_vel_nom
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import math
from scipy.linalg import solve_discrete_are

class LQRNode(Node):
    def __init__(self):
        super().__init__('lqr_node')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nom', 10)
        
        # Subscribers
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.create_subscription(PoseStamped, '/state', self.state_callback, 10)
        
        # Parameters
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('lqr.q_x', 1.0)
        self.declare_parameter('lqr.q_y', 1.0)
        self.declare_parameter('lqr.q_theta', 2.0)
        self.declare_parameter('lqr.r_v', 0.1)
        self.declare_parameter('lqr.r_w', 0.1)
        self.declare_parameter('lqr.lookahead_distance', 2.0)
        self.declare_parameter('vehicle.max_speed', 5.0)
        self.declare_parameter('vehicle.max_angular_velocity', math.pi / 2)
        
        # Get parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.q_x = self.get_parameter('lqr.q_x').value
        self.q_y = self.get_parameter('lqr.q_y').value
        self.q_theta = self.get_parameter('lqr.q_theta').value
        self.r_v = self.get_parameter('lqr.r_v').value
        self.r_w = self.get_parameter('lqr.r_w').value
        self.lookahead_dist = self.get_parameter('lqr.lookahead_distance').value
        self.v_max = self.get_parameter('vehicle.max_speed').value
        self.w_max = self.get_parameter('vehicle.max_angular_velocity').value
        
        # Cost matrices
        self.Q = np.diag([self.q_x, self.q_y, self.q_theta])
        self.R = np.diag([self.r_v, self.r_w])
        
        # State variables
        self.current_state = None
        self.planned_path = None
        
        # Control timer
        self.dt = 1.0 / self.control_rate
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('LQR Controller Node initialized')
    
    def path_callback(self, msg):
        """Store the planned path"""
        self.planned_path = msg
        self.get_logger().info(f'Received path with {len(msg.poses)} waypoints')
    
    def state_callback(self, msg):
        """Store current state"""
        self.current_state = msg
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def find_lookahead_point(self, current_pos, path_poses):
        """Find lookahead point on path"""
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
        cumulative_dist = 0.0
        
        for i in range(closest_idx, len(path_poses) - 1):
            p1 = path_poses[i]
            p2 = path_poses[i + 1]
            segment_dist = math.sqrt(
                (p2.pose.position.x - p1.pose.position.x)**2 +
                (p2.pose.position.y - p1.pose.position.y)**2
            )
            cumulative_dist += segment_dist
            
            if cumulative_dist >= self.lookahead_dist:
                lookahead_idx = i + 1
                break
        
        # If at end of path, use last point
        if lookahead_idx >= len(path_poses) - 1:
            lookahead_idx = len(path_poses) - 1
        
        return lookahead_idx, min_dist
    
    def compute_lqr_control(self, state, ref_state, ref_control):
        """
        Compute LQR control for differential drive
        state: [x, y, theta]
        ref_state: [x_ref, y_ref, theta_ref]
        ref_control: [v_ref, w_ref]
        Returns: (v_cmd, w_cmd)
        """
        x, y, theta = state
        x_ref, y_ref, theta_ref = ref_state
        v_ref, w_ref = ref_control
        
        # Linearize system around reference
        # State: [x, y, theta]
        # Control: [v, w]
        # dx/dt = A*x + B*u
        
        A = np.array([
            [0, 0, -v_ref * math.sin(theta_ref)],
            [0, 0,  v_ref * math.cos(theta_ref)],
            [0, 0,  0]
        ])
        
        B = np.array([
            [math.cos(theta_ref), 0],
            [math.sin(theta_ref), 0],
            [0, 1]
        ])
        
        # Discretize system
        Ad = np.eye(3) + A * self.dt
        Bd = B * self.dt
        
        # Solve discrete-time algebraic Riccati equation
        try:
            P = solve_discrete_are(Ad, Bd, self.Q, self.R)
            K = np.linalg.inv(self.R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
        except:
            # Fallback to simple gains if DARE fails
            self.get_logger().warn('DARE failed, using fallback gains')
            K = np.array([[1.0, 0.5, 0.3], [0.0, 0.0, 2.0]])
        
        # State error
        x_err = np.array([
            x - x_ref,
            y - y_ref,
            self.normalize_angle(theta - theta_ref)
        ])
        
        # Compute control
        u = -K @ x_err + np.array([v_ref, w_ref])
        
        # Apply control limits
        v_cmd = max(min(u[0], self.v_max), 0.0)
        w_cmd = max(min(u[1], self.w_max), -self.w_max)
        
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
        x_ref = lookahead_pose.pose.position.x
        y_ref = lookahead_pose.pose.position.y
        theta_ref = self.quaternion_to_yaw(lookahead_pose.pose.orientation)
        
        # Compute desired velocity towards lookahead point
        dx = x_ref - x
        dy = y_ref - y
        dist_to_target = math.sqrt(dx**2 + dy**2)
        
        # Reference control
        v_ref = min(2.0, dist_to_target)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - theta)
        w_ref = 0.5 * angle_diff
        
        # Check if reached goal (last point in path)
        if lookahead_idx == len(self.planned_path.poses) - 1:
            goal_dist = math.sqrt((x_ref - x)**2 + (y_ref - y)**2)
            if goal_dist < 0.15:  # Goal tolerance
                # Reached goal, stop
                cmd = Twist()
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
                self.get_logger().info('Goal reached!', throttle_duration_sec=2.0)
                return
        
        # Compute LQR control command
        state = np.array([x, y, theta])
        ref_state = np.array([x_ref, y_ref, theta_ref])
        ref_control = np.array([v_ref, w_ref])
        v_cmd, w_cmd = self.compute_lqr_control(state, ref_state, ref_control)
        
        # Log for debugging
        self.get_logger().info(
            f'LQR Control: v={v_cmd:.2f}, w={w_cmd:.2f}, lookahead_idx={lookahead_idx}/{len(self.planned_path.poses)}',
            throttle_duration_sec=1.0
        )
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LQRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
