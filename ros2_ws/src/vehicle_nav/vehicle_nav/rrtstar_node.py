#!/usr/bin/env python3
"""
RRT* Planner Node
Subscribes to: /goal (geometry_msgs/PoseStamped), /state (geometry_msgs/PoseStamped), /obstacles (visualization_msgs/MarkerArray)
Publishes to: /planned_path (nav_msgs/Path)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
import numpy as np
import time


class RRTStarNode(Node):
    def __init__(self):
        super().__init__('rrtstar_planner')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal',
            self.goal_callback,
            10
        )
        self.state_sub = self.create_subscription(
            PoseStamped,
            '/state',
            self.state_callback,
            10
        )
        self.obstacle_sub = self.create_subscription(
            MarkerArray,
            '/obstacles',
            self.obstacle_callback,
            10
        )
        
        # Parameters
        self.declare_parameter('rrtstar.max_iterations', 3000)
        self.declare_parameter('rrtstar.step_size', 2.0)
        self.declare_parameter('rrtstar.search_radius', 5.0)
        self.declare_parameter('rrtstar.goal_sample_rate', 0.1)
        self.declare_parameter('rrtstar.goal_threshold', 1.5)
        self.declare_parameter('environment.x_min', -10.0)
        self.declare_parameter('environment.x_max', 10.0)
        self.declare_parameter('environment.y_min', -10.0)
        self.declare_parameter('environment.y_max', 10.0)
        
        # Get parameters
        self.max_iterations = self.get_parameter('rrtstar.max_iterations').value
        self.step_size = self.get_parameter('rrtstar.step_size').value
        self.search_radius = self.get_parameter('rrtstar.search_radius').value
        self.goal_sample_rate = self.get_parameter('rrtstar.goal_sample_rate').value
        self.goal_threshold = self.get_parameter('rrtstar.goal_threshold').value
        self.x_min = self.get_parameter('environment.x_min').value
        self.x_max = self.get_parameter('environment.x_max').value
        self.y_min = self.get_parameter('environment.y_min').value
        self.y_max = self.get_parameter('environment.y_max').value
        
        # State
        self.current_state = None
        self.goal = None
        self.obstacles = []
        
        # RRT* tree
        self.nodes = []
        self.parents = []
        self.costs = []
        
        self.get_logger().info('RRT* Planner Node initialized')
    
    def goal_callback(self, msg):
        """Handle new goal"""
        self.goal = msg
        self.get_logger().info(f'Received goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.plan_path()
    
    def state_callback(self, msg):
        """Update current state"""
        self.current_state = msg
    
    def obstacle_callback(self, msg):
        """Update obstacles"""
        self.obstacles = []
        for marker in msg.markers:
            obs = {
                'x': marker.pose.position.x,
                'y': marker.pose.position.y,
                'radius': marker.scale.x / 2.0  # Assuming cylindrical markers
            }
            self.obstacles.append(obs)
        self.get_logger().info(f'Received {len(self.obstacles)} obstacles')
    
    def plan_path(self):
        """Plan path using RRT*"""
        if self.current_state is None or self.goal is None:
            self.get_logger().warn('Cannot plan: missing state or goal')
            return
        
        # Extract start and goal positions
        start = np.array([
            self.current_state.pose.position.x,
            self.current_state.pose.position.y
        ])
        goal = np.array([
            self.goal.pose.position.x,
            self.goal.pose.position.y
        ])
        
        self.get_logger().info(f'Planning path from ({start[0]:.2f}, {start[1]:.2f}) to ({goal[0]:.2f}, {goal[1]:.2f})')
        
        # Run RRT*
        start_time = time.time()
        path, stats = self.rrtstar_search(start, goal)
        runtime = time.time() - start_time
        
        if stats['success']:
            self.get_logger().info(
                f"RRT* succeeded: {stats['nodes_created']} nodes, "
                f"{stats['iterations']} iterations, "
                f"path length: {stats['path_length']:.2f}m, "
                f"runtime: {runtime:.3f}s"
            )
            self.publish_path(path)
        else:
            self.get_logger().error(
                f"RRT* failed after {stats['iterations']} iterations, "
                f"{stats['nodes_created']} nodes created"
            )
    
    def rrtstar_search(self, start, goal):
        """RRT* algorithm implementation"""
        # Initialize tree
        self.nodes = [start]
        self.parents = [0]
        self.costs = [0.0]
        
        goal_node_idx = -1
        best_cost = float('inf')
        iterations = 0
        
        # RRT* main loop
        for iter in range(self.max_iterations):
            iterations = iter + 1
            
            # Sample random point (with goal bias)
            if np.random.random() < self.goal_sample_rate:
                x_rand = goal
            else:
                x_rand = self.sample_random_point()
            
            # Find nearest node
            nearest_idx = self.find_nearest(x_rand)
            x_nearest = self.nodes[nearest_idx]
            
            # Steer towards random point
            x_new = self.steer(x_nearest, x_rand)
            
            # Check collision
            if self.is_collision_free(x_nearest, x_new):
                # Find nearby nodes for rewiring
                near_indices = self.find_near(x_new)
                
                # Choose best parent
                min_cost, min_idx = self.choose_parent(x_new, nearest_idx, near_indices)
                
                # Add new node
                new_idx = len(self.nodes)
                self.nodes.append(x_new)
                self.parents.append(min_idx)
                self.costs.append(min_cost)
                
                # Rewire tree
                self.rewire(new_idx, near_indices)
                
                # Check if goal reached
                if np.linalg.norm(x_new - goal) < self.goal_threshold:
                    current_cost = self.costs[new_idx] + np.linalg.norm(x_new - goal)
                    if current_cost < best_cost:
                        goal_node_idx = new_idx
                        best_cost = current_cost
            
            # Early termination if good path found
            if goal_node_idx >= 0 and iter > 1000:
                break
        
        # Extract path
        if goal_node_idx >= 0:
            path = self.extract_path(goal_node_idx, goal)
            stats = {
                'iterations': iterations,
                'nodes_created': len(self.nodes),
                'path_length': self.compute_path_length(path),
                'success': True
            }
        else:
            path = np.array([])
            stats = {
                'iterations': iterations,
                'nodes_created': len(self.nodes),
                'path_length': float('inf'),
                'success': False
            }
        
        return path, stats
    
    def sample_random_point(self):
        """Sample random point in environment"""
        x = np.random.uniform(self.x_min, self.x_max)
        y = np.random.uniform(self.y_min, self.y_max)
        return np.array([x, y])
    
    def find_nearest(self, x):
        """Find nearest node in tree to point x"""
        distances = [np.linalg.norm(node - x) for node in self.nodes]
        return np.argmin(distances)
    
    def steer(self, x_from, x_to):
        """Steer from x_from towards x_to with step_size limit"""
        direction = x_to - x_from
        distance = np.linalg.norm(direction)
        
        if distance < self.step_size:
            return x_to.copy()
        else:
            return x_from + (direction / distance) * self.step_size
    
    def is_collision_free(self, x_from, x_to):
        """Check if path from x_from to x_to is collision free"""
        # Sample points along the path more densely
        num_checks = max(10, int(np.ceil(np.linalg.norm(x_to - x_from) / 0.2)))
        
        for i in range(num_checks + 1):
            t = i / max(num_checks, 1)
            x_check = x_from + t * (x_to - x_from)
            
            if self.check_collision(x_check):
                return False
        
        return True
    
    def check_collision(self, pos):
        """Check if position collides with any obstacle"""
        # Add safety margin (vehicle radius + obstacle radius + buffer)
        safety_margin = 1.5  # meters
        for obs in self.obstacles:
            dist = np.linalg.norm(pos - np.array([obs['x'], obs['y']]))
            if dist < (obs['radius'] + safety_margin):
                return True
        return False
    
    def find_near(self, x):
        """Find all nodes within search_radius of x"""
        near_indices = []
        for i, node in enumerate(self.nodes):
            if np.linalg.norm(node - x) < self.search_radius:
                near_indices.append(i)
        return near_indices
    
    def choose_parent(self, x_new, nearest_idx, near_indices):
        """Choose best parent from nearby nodes"""
        min_cost = self.costs[nearest_idx] + np.linalg.norm(self.nodes[nearest_idx] - x_new)
        min_idx = nearest_idx
        
        for idx in near_indices:
            potential_cost = self.costs[idx] + np.linalg.norm(self.nodes[idx] - x_new)
            
            if potential_cost < min_cost:
                if self.is_collision_free(self.nodes[idx], x_new):
                    min_cost = potential_cost
                    min_idx = idx
        
        return min_cost, min_idx
    
    def rewire(self, new_idx, near_indices):
        """Rewire tree through new node if it provides better path"""
        x_new = self.nodes[new_idx]
        
        for idx in near_indices:
            if idx == new_idx:
                continue
            
            x_near = self.nodes[idx]
            potential_cost = self.costs[new_idx] + np.linalg.norm(x_new - x_near)
            
            if potential_cost < self.costs[idx]:
                if self.is_collision_free(x_new, x_near):
                    self.parents[idx] = new_idx
                    self.costs[idx] = potential_cost
    
    def extract_path(self, goal_idx, goal):
        """Extract path from start to goal"""
        path = [goal]
        current_idx = goal_idx
        
        # Trace back through parents to start
        while current_idx > 0:
            path.insert(0, self.nodes[current_idx])
            current_idx = self.parents[current_idx]
        
        # Add start node (index 0)
        path.insert(0, self.nodes[0])
        
        return np.array(path)
    
    def compute_path_length(self, path):
        """Compute total path length"""
        if len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            length += np.linalg.norm(path[i+1] - path[i])
        return length
    
    def publish_path(self, waypoints):
        """Publish path as nav_msgs/Path"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for i, wp in enumerate(waypoints):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(wp[0])
            pose.pose.position.y = float(wp[1])
            pose.pose.position.z = 0.0
            
            # Compute orientation from path direction
            if i < len(waypoints) - 1:
                # Use direction to next waypoint
                dx = waypoints[i+1][0] - wp[0]
                dy = waypoints[i+1][1] - wp[1]
                theta = np.arctan2(dy, dx)
            else:
                # Last point: use direction from previous waypoint
                if i > 0:
                    dx = wp[0] - waypoints[i-1][0]
                    dy = wp[1] - waypoints[i-1][1]
                    theta = np.arctan2(dy, dx)
                else:
                    theta = 0.0
            
            # Convert theta to quaternion
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = np.sin(theta / 2.0)
            pose.pose.orientation.w = np.cos(theta / 2.0)
            
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published path with {len(waypoints)} waypoints')


def main(args=None):
    rclpy.init(args=args)
    node = RRTStarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
