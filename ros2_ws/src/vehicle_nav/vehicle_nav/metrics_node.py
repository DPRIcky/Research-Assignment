#!/usr/bin/env python3
"""
Metrics Collection Node
Collects performance metrics during testing
Outputs: tracking error, path length, control effort, computation time, etc.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
import numpy as np
import math
import time
import json
from datetime import datetime

class MetricsNode(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        
        # Subscribers
        self.create_subscription(PoseStamped, '/state', self.state_callback, 10)
        self.create_subscription(PoseStamped, '/goal', self.goal_callback, 10)
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_nom', self.cmd_nom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_safe', self.cmd_safe_callback, 10)
        self.create_subscription(MarkerArray, '/obstacles', self.obstacles_callback, 10)
        
        # Parameters
        self.declare_parameter('test_name', 'test')
        self.declare_parameter('timeout', 60.0)  # Max time per test (seconds)
        self.declare_parameter('goal_tolerance', 0.5)  # Goal reached tolerance
        self.declare_parameter('output_file', '')  # Output JSON file
        
        self.test_name = self.get_parameter('test_name').value
        self.timeout = self.get_parameter('timeout').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.output_file = self.get_parameter('output_file').value
        
        # State
        self.current_state = None
        self.goal = None
        self.planned_path = None
        self.obstacles = []
        self.trajectory = []  # Actual trajectory
        self.start_time = None
        self.end_time = None
        self.goal_reached = False
        
        # Metrics
        self.tracking_errors = []
        self.control_efforts = []
        self.cbf_active = False
        self.cbf_interventions = 0
        self.safety_violations = 0
        self.planning_time = None
        
        # Create timer to check goal and timeout
        self.create_timer(0.1, self.check_status)
        
        self.get_logger().info(f'Metrics collector initialized for test: {self.test_name}')
    
    def state_callback(self, msg):
        """Record state"""
        self.current_state = msg
        
        # Record trajectory
        pos = (msg.pose.position.x, msg.pose.position.y)
        timestamp = time.time()
        
        if len(self.trajectory) == 0:
            # First state - start timer
            self.start_time = timestamp
            self.get_logger().info('Test started')
        
        self.trajectory.append({
            'x': pos[0],
            'y': pos[1],
            'time': timestamp
        })
        
        # Compute tracking error if we have a path
        if self.planned_path is not None and len(self.planned_path.poses) > 0:
            tracking_error = self.compute_tracking_error(pos)
            self.tracking_errors.append(tracking_error)
        
        # Check for safety violations
        if self.check_collision(pos):
            self.safety_violations += 1
    
    def goal_callback(self, msg):
        """Store goal"""
        self.goal = msg
        self.get_logger().info(f'Goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
    
    def path_callback(self, msg):
        """Store planned path"""
        self.planned_path = msg
        
        # Record planning time (time from start to first path)
        if self.planning_time is None and self.start_time is not None:
            self.planning_time = time.time() - self.start_time
            self.get_logger().info(f'Planning time: {self.planning_time:.3f}s')
    
    def cmd_nom_callback(self, msg):
        """Track nominal control effort"""
        effort = abs(msg.linear.x) + abs(msg.angular.z)
        self.control_efforts.append(effort)
    
    def cmd_safe_callback(self, msg):
        """Track if CBF is active"""
        self.cbf_active = True
        # CBF interventions tracked by comparing to nominal (done in cbf_node logs)
    
    def obstacles_callback(self, msg):
        """Store obstacles"""
        self.obstacles = []
        for marker in msg.markers:
            obs = {
                'x': marker.pose.position.x,
                'y': marker.pose.position.y,
                'radius': marker.scale.x / 2.0
            }
            self.obstacles.append(obs)
    
    def compute_tracking_error(self, pos):
        """Compute distance to nearest point on planned path"""
        min_dist = float('inf')
        
        for pose in self.planned_path.poses:
            path_x = pose.pose.position.x
            path_y = pose.pose.position.y
            dist = math.sqrt((pos[0] - path_x)**2 + (pos[1] - path_y)**2)
            min_dist = min(min_dist, dist)
        
        return min_dist
    
    def check_collision(self, pos):
        """Check if position collides with any obstacle"""
        for obs in self.obstacles:
            dist = math.sqrt((pos[0] - obs['x'])**2 + (pos[1] - obs['y'])**2)
            if dist < obs['radius']:
                return True
        return False
    
    def check_status(self):
        """Check if goal reached or timeout"""
        if self.current_state is None or self.goal is None:
            return
        
        # Check goal reached
        if not self.goal_reached:
            x = self.current_state.pose.position.x
            y = self.current_state.pose.position.y
            gx = self.goal.pose.position.x
            gy = self.goal.pose.position.y
            
            dist_to_goal = math.sqrt((x - gx)**2 + (y - gy)**2)
            
            if dist_to_goal < self.goal_tolerance:
                self.goal_reached = True
                self.end_time = time.time()
                self.get_logger().info(f'Goal reached in {self.end_time - self.start_time:.2f}s')
                self.save_metrics()
                # Shutdown node
                rclpy.shutdown()
        
        # Check timeout
        if self.start_time is not None and not self.goal_reached:
            elapsed = time.time() - self.start_time
            if elapsed > self.timeout:
                self.end_time = time.time()
                self.get_logger().warn(f'Timeout reached ({self.timeout}s), goal not reached')
                self.save_metrics()
                # Shutdown node
                rclpy.shutdown()
    
    def compute_path_length(self, trajectory):
        """Compute total path length from trajectory"""
        if len(trajectory) < 2:
            return 0.0
        
        length = 0.0
        for i in range(1, len(trajectory)):
            dx = trajectory[i]['x'] - trajectory[i-1]['x']
            dy = trajectory[i]['y'] - trajectory[i-1]['y']
            length += math.sqrt(dx**2 + dy**2)
        
        return length
    
    def save_metrics(self):
        """Compute and save final metrics"""
        metrics = {
            'test_name': self.test_name,
            'timestamp': datetime.now().isoformat(),
            'goal_reached': self.goal_reached,
            'execution_time': self.end_time - self.start_time if self.start_time and self.end_time else 0.0,
            'planning_time': self.planning_time if self.planning_time else 0.0,
            'path_length': self.compute_path_length(self.trajectory),
            'mean_tracking_error': float(np.mean(self.tracking_errors)) if len(self.tracking_errors) > 0 else 0.0,
            'max_tracking_error': float(np.max(self.tracking_errors)) if len(self.tracking_errors) > 0 else 0.0,
            'std_tracking_error': float(np.std(self.tracking_errors)) if len(self.tracking_errors) > 0 else 0.0,
            'mean_control_effort': float(np.mean(self.control_efforts)) if len(self.control_efforts) > 0 else 0.0,
            'cbf_active': self.cbf_active,
            'safety_violations': self.safety_violations,
            'num_trajectory_points': len(self.trajectory)
        }
        
        # Print metrics
        self.get_logger().info('='*60)
        self.get_logger().info(f'METRICS for {self.test_name}:')
        self.get_logger().info(f'  Goal Reached: {metrics["goal_reached"]}')
        self.get_logger().info(f'  Execution Time: {metrics["execution_time"]:.2f}s')
        self.get_logger().info(f'  Planning Time: {metrics["planning_time"]:.3f}s')
        self.get_logger().info(f'  Path Length: {metrics["path_length"]:.2f}m')
        self.get_logger().info(f'  Mean Tracking Error: {metrics["mean_tracking_error"]:.3f}m')
        self.get_logger().info(f'  Max Tracking Error: {metrics["max_tracking_error"]:.3f}m')
        self.get_logger().info(f'  Mean Control Effort: {metrics["mean_control_effort"]:.3f}')
        self.get_logger().info(f'  Safety Violations: {metrics["safety_violations"]}')
        self.get_logger().info('='*60)
        
        # Save to file if specified
        if self.output_file:
            with open(self.output_file, 'w') as f:
                json.dump(metrics, f, indent=2)
            self.get_logger().info(f'Metrics saved to {self.output_file}')

def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
