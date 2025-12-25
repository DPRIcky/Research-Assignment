#!/usr/bin/env python3
"""
A* Path Planner Node
Subscribes to: /goal (geometry_msgs/PoseStamped), /state (geometry_msgs/PoseStamped)
Publishes to: /planned_path (nav_msgs/Path)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
import numpy as np
from math import sqrt, sin, cos
import heapq
import time


class AStarNode(Node):
    def __init__(self):
        super().__init__('astar_planner')
        
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
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('x_min', -10.0)
        self.declare_parameter('x_max', 10.0)
        self.declare_parameter('y_min', -10.0)
        self.declare_parameter('y_max', 10.0)
        
        self.resolution = self.get_parameter('grid_resolution').value
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        
        # State variables
        self.current_state = None
        self.obstacles = []
        self.obstacle_margin = 0.5  # Safety margin around obstacles
        
        self.get_logger().info('A* Planner Node initialized')
        self.get_logger().info(f'Grid resolution: {self.resolution}m')
    
    def state_callback(self, msg):
        """Update current state"""
        self.current_state = msg
    
    def obstacle_callback(self, msg):
        """Update obstacle list"""
        self.obstacles = []
        for marker in msg.markers:
            obstacle = {
                'x': marker.pose.position.x,
                'y': marker.pose.position.y,
                'radius': marker.scale.x / 2.0 + self.obstacle_margin
            }
            self.obstacles.append(obstacle)
    
    def goal_callback(self, msg):
        """Handle goal position and compute A* path"""
        if self.current_state is None:
            self.get_logger().warn('No current state available yet')
            return
        
        start = np.array([self.current_state.pose.position.x,
                         self.current_state.pose.position.y])
        goal = np.array([msg.pose.position.x,
                        msg.pose.position.y])
        
        self.get_logger().info(f'Planning from ({start[0]:.2f}, {start[1]:.2f}) to ({goal[0]:.2f}, {goal[1]:.2f})')
        
        # Run A* search
        path, stats = self.astar_search(start, goal)
        
        if stats['success']:
            self.get_logger().info(f'Path found! Length: {stats["path_length"]:.2f}m, '
                                 f'Nodes: {stats["nodes_expanded"]}, Time: {stats["runtime"]:.3f}s')
            self.publish_path(path)
        else:
            self.get_logger().warn('No path found!')
    
    def astar_search(self, start, goal):
        """A* search algorithm"""
        start_time = time.time()
        
        # Check if start/goal are in collision
        if self.check_collision(start):
            return None, {'success': False, 'nodes_expanded': 0, 'path_length': 0, 'runtime': 0}
        if self.check_collision(goal):
            self.get_logger().warn('Goal is in collision!')
            return None, {'success': False, 'nodes_expanded': 0, 'path_length': 0, 'runtime': 0}
        
        # Convert to grid coordinates
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)
        
        # Initialize open and closed sets
        open_set = []  # Priority queue: (f_score, counter, node)
        open_dict = {}  # For quick lookup: key -> (g_score, parent)
        closed_set = set()
        counter = 0
        
        # Add start node
        start_key = tuple(start_grid)
        h = self.heuristic(start_grid, goal_grid)
        heapq.heappush(open_set, (h, counter, start_key))
        open_dict[start_key] = {'g': 0, 'parent': None}
        counter += 1
        
        nodes_expanded = 0
        
        # A* main loop
        while open_set:
            # Get node with lowest f-score
            f, _, current_key = heapq.heappop(open_set)
            
            if current_key in closed_set:
                continue
            
            current = np.array(current_key)
            current_g = open_dict[current_key]['g']
            
            # Check if goal reached
            if np.array_equal(current, goal_grid):
                path = self.reconstruct_path(open_dict, current_key)
                runtime = time.time() - start_time
                path_length = self.compute_path_length(path)
                return path, {
                    'success': True,
                    'nodes_expanded': nodes_expanded,
                    'path_length': path_length,
                    'runtime': runtime
                }
            
            closed_set.add(current_key)
            nodes_expanded += 1
            
            # Expand neighbors
            for neighbor in self.get_neighbors(current):
                neighbor_key = tuple(neighbor)
                
                if neighbor_key in closed_set:
                    continue
                
                # Check if within bounds
                if not self.in_bounds(neighbor):
                    continue
                
                # Check collision
                world_pos = self.grid_to_world(neighbor)
                if self.check_collision(world_pos):
                    continue
                
                # Calculate tentative g-score
                move_cost = np.linalg.norm(neighbor - current) * self.resolution
                tentative_g = current_g + move_cost
                
                # Check if this path is better
                if neighbor_key not in open_dict or tentative_g < open_dict[neighbor_key]['g']:
                    open_dict[neighbor_key] = {'g': tentative_g, 'parent': current_key}
                    h = self.heuristic(neighbor, goal_grid)
                    f = tentative_g + h
                    heapq.heappush(open_set, (f, counter, neighbor_key))
                    counter += 1
        
        # No path found
        runtime = time.time() - start_time
        return None, {
            'success': False,
            'nodes_expanded': nodes_expanded,
            'path_length': float('inf'),
            'runtime': runtime
        }
    
    def reconstruct_path(self, open_dict, goal_key):
        """Reconstruct path from goal to start"""
        path = []
        current_key = goal_key
        
        while current_key is not None:
            world_pos = self.grid_to_world(np.array(current_key))
            path.append(world_pos)
            current_key = open_dict[current_key]['parent']
        
        path.reverse()
        return np.array(path)
    
    def heuristic(self, pos1, pos2):
        """Manhattan distance heuristic"""
        dx = abs(pos1[0] - pos2[0])
        dy = abs(pos1[1] - pos2[1])
        return (dx + dy) * self.resolution
    
    def get_neighbors(self, pos):
        """Get 8-connected neighbors"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = pos + np.array([dx, dy])
                neighbors.append(neighbor)
        return neighbors
    
    def world_to_grid(self, pos):
        """Convert world coordinates to grid coordinates"""
        grid_x = int((pos[0] - self.x_min) / self.resolution)
        grid_y = int((pos[1] - self.y_min) / self.resolution)
        return np.array([grid_x, grid_y])
    
    def grid_to_world(self, grid_pos):
        """Convert grid coordinates to world coordinates"""
        world_x = grid_pos[0] * self.resolution + self.x_min
        world_y = grid_pos[1] * self.resolution + self.y_min
        return np.array([world_x, world_y])
    
    def in_bounds(self, grid_pos):
        """Check if grid position is within bounds"""
        world_pos = self.grid_to_world(grid_pos)
        return (self.x_min <= world_pos[0] <= self.x_max and
                self.y_min <= world_pos[1] <= self.y_max)
    
    def check_collision(self, pos):
        """Check if position collides with any obstacle"""
        for obs in self.obstacles:
            dist = np.linalg.norm(pos - np.array([obs['x'], obs['y']]))
            if dist < obs['radius']:
                return True
        return False
    
    def compute_path_length(self, path):
        """Compute total path length"""
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
    node = AStarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
