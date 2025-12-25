#!/usr/bin/env python3
"""
TurtleSim Bridge Node
Bridges our navigation system with turtlesim for visualization
- Main turtle represents the vehicle
- Additional turtles represent obstacles
- Draws planned path using pen
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from turtlesim.msg import Pose as TurtlePose
from turtlesim.srv import Spawn, Kill, SetPen, TeleportAbsolute
from visualization_msgs.msg import MarkerArray
import math

class TurtleSimBridge(Node):
    def __init__(self):
        super().__init__('turtlesim_bridge')
        
        # State tracking
        self.current_state = None
        self.obstacles_spawned = False
        self.path_drawn = False
        self.current_path = None
        
        # Turtlesim coordinate system: 0-11 (with origin at bottom-left)
        # Our system: -10 to 10 (origin at center)
        # Scaling: turtlesim = (our + 10) * 11/20 = (our + 10) * 0.55
        self.scale = 0.55
        self.offset = 10.0
        
        # Subscribe to our topics
        self.create_subscription(Twist, '/cmd_vel_nom', self.cmd_callback, 10)
        self.create_subscription(PoseStamped, '/state', self.state_callback, 10)
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.create_subscription(MarkerArray, '/obstacles', self.obstacles_callback, 10)
        self.create_subscription(TurtlePose, '/turtle1/pose', self.turtle_pose_callback, 10)
        
        # Publish to turtlesim
        self.turtle_cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Publish state back to our system (from turtlesim pose)
        self.state_pub = self.create_publisher(PoseStamped, '/state', 10)
        
        # Service clients for turtlesim
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        
        # Wait for turtlesim services
        self.get_logger().info('Waiting for turtlesim services...')
        self.spawn_client.wait_for_service(timeout_sec=5.0)
        
        # Initialize turtle at starting position
        self.create_timer(0.5, self.initialize_turtle, one_shot=True)
        
        self.get_logger().info('TurtleSim Bridge initialized')
    
    def world_to_turtle(self, x, y):
        """Convert our coordinate system to turtlesim coordinates"""
        tx = (x + self.offset) * self.scale
        ty = (y + self.offset) * self.scale
        return tx, ty
    
    def turtle_to_world(self, tx, ty):
        """Convert turtlesim coordinates to our coordinate system"""
        x = (tx / self.scale) - self.offset
        y = (ty / self.scale) - self.offset
        return x, y
    
    def initialize_turtle(self):
        """Initialize turtle at starting position"""
        # Turn off pen initially
        pen_req = SetPen.Request()
        pen_req.off = 1
        self.pen_client.call_async(pen_req)
        
        # Teleport to start position (0, 0 in our coords)
        tx, ty = self.world_to_turtle(0.0, 0.0)
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = tx
        teleport_req.y = ty
        teleport_req.theta = 0.0
        self.teleport_client.call_async(teleport_req)
        
        self.get_logger().info('Turtle initialized at origin')
    
    def turtle_pose_callback(self, msg):
        """Convert turtlesim pose to our state and publish"""
        # Convert coordinates
        x, y = self.turtle_to_world(msg.x, msg.y)
        
        # Create PoseStamped message
        state_msg = PoseStamped()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = 'map'
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = 0.0
        
        # Convert theta to quaternion
        state_msg.pose.orientation.x = 0.0
        state_msg.pose.orientation.y = 0.0
        state_msg.pose.orientation.z = math.sin(msg.theta / 2.0)
        state_msg.pose.orientation.w = math.cos(msg.theta / 2.0)
        
        self.state_pub.publish(state_msg)
        self.current_state = state_msg
    
    def cmd_callback(self, msg):
        """Forward control commands to turtlesim"""
        self.turtle_cmd_pub.publish(msg)
    
    def state_callback(self, msg):
        """Store current state"""
        self.current_state = msg
    
    def path_callback(self, msg):
        """Draw planned path in turtlesim"""
        if len(msg.poses) == 0:
            return
        
        self.current_path = msg
        
        # Draw path after a short delay
        self.create_timer(0.1, self.draw_path, one_shot=True)
    
    def draw_path(self):
        """Draw the planned path using turtle pen"""
        if self.current_path is None or len(self.current_path.poses) == 0:
            return
        
        # Save current position
        # Turn off pen
        pen_req = SetPen.Request()
        pen_req.off = 1
        self.pen_client.call_async(pen_req)
        
        # Teleport to first point
        first = self.current_path.poses[0]
        tx, ty = self.world_to_turtle(first.pose.position.x, first.pose.position.y)
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = tx
        teleport_req.y = ty
        teleport_req.theta = 0.0
        self.teleport_client.call_async(teleport_req)
        
        # Turn on pen (green for path)
        def enable_pen():
            pen_req = SetPen.Request()
            pen_req.r = 0
            pen_req.g = 255
            pen_req.b = 0
            pen_req.width = 2
            pen_req.off = 0
            self.pen_client.call_async(pen_req)
            
            # Start drawing after pen is enabled
            self.create_timer(0.1, self.draw_path_points, one_shot=True)
        
        self.create_timer(0.1, enable_pen, one_shot=True)
    
    def draw_path_points(self):
        """Draw path by teleporting through points"""
        if self.current_path is None:
            return
        
        # Draw every 5th point to speed up
        step = max(1, len(self.current_path.poses) // 50)
        
        def teleport_next(index):
            if index >= len(self.current_path.poses):
                # Done drawing, turn off pen and return to vehicle position
                pen_req = SetPen.Request()
                pen_req.off = 1
                self.pen_client.call_async(pen_req)
                
                # Return to current position
                if self.current_state:
                    tx, ty = self.world_to_turtle(
                        self.current_state.pose.position.x,
                        self.current_state.pose.position.y
                    )
                    teleport_req = TeleportAbsolute.Request()
                    teleport_req.x = tx
                    teleport_req.y = ty
                    # Extract theta from quaternion
                    q = self.current_state.pose.orientation
                    theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    teleport_req.theta = theta
                    self.teleport_client.call_async(teleport_req)
                
                self.get_logger().info('Path drawn!')
                return
            
            # Teleport to this point
            pose = self.current_path.poses[index]
            tx, ty = self.world_to_turtle(pose.pose.position.x, pose.pose.position.y)
            teleport_req = TeleportAbsolute.Request()
            teleport_req.x = tx
            teleport_req.y = ty
            teleport_req.theta = 0.0
            self.teleport_client.call_async(teleport_req)
            
            # Schedule next point
            self.create_timer(0.01, lambda: teleport_next(index + step), one_shot=True)
        
        # Start teleporting through points
        teleport_next(0)
    
    def obstacles_callback(self, msg):
        """Spawn obstacle turtles"""
        if self.obstacles_spawned or len(msg.markers) == 0:
            return
        
        self.obstacles_spawned = True
        
        # Spawn a turtle for each obstacle (red circles)
        for i, marker in enumerate(msg.markers):
            tx, ty = self.world_to_turtle(marker.pose.position.x, marker.pose.position.y)
            
            # Spawn turtle
            spawn_req = Spawn.Request()
            spawn_req.x = tx
            spawn_req.y = ty
            spawn_req.theta = 0.0
            spawn_req.name = f'obstacle_{i}'
            
            future = self.spawn_client.call_async(spawn_req)
            
            # Set pen to draw red circle
            def set_obstacle_pen(obs_name):
                pen_client = self.create_client(SetPen, f'/{obs_name}/set_pen')
                pen_client.wait_for_service(timeout_sec=1.0)
                pen_req = SetPen.Request()
                pen_req.r = 255
                pen_req.g = 0
                pen_req.b = 0
                pen_req.width = 3
                pen_req.off = 0
                pen_client.call_async(pen_req)
            
            self.create_timer(0.2 * (i + 1), lambda name=spawn_req.name: set_obstacle_pen(name), one_shot=True)
        
        self.get_logger().info(f'Spawned {len(msg.markers)} obstacle turtles')


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSimBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
