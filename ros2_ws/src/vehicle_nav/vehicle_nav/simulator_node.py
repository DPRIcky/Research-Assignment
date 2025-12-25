#!/usr/bin/env python3
"""
Vehicle Simulator Node
Simulates vehicle dynamics and publishes state
Mimics MATLAB simulation environment
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import numpy as np
from math import sin, cos, atan2, sqrt


class SimulatorNode(Node):
    def __init__(self):
        super().__init__('vehicle_simulator')
        
        # Publishers
        self.state_pub = self.create_publisher(PoseStamped, '/state', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.traj_pub = self.create_publisher(Path, '/trajectory_history', 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, '/obstacles', 10)
        self.vehicle_marker_pub = self.create_publisher(Marker, '/vehicle_marker', 10)
        
        # TF broadcaster for robot visualization
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber for control commands
        # Subscribe to both nominal and safe commands
        # When CBF is not active, /cmd_vel_nom will be used
        # When CBF is active, /cmd_vel_safe will be used
        self.cmd_sub_safe = self.create_subscription(
            Twist,
            '/cmd_vel_safe',
            self.cmd_safe_callback,
            10
        )
        self.cmd_sub_nom = self.create_subscription(
            Twist,
            '/cmd_vel_nom',
            self.cmd_nom_callback,
            10
        )
        
        # Parameters
        self.declare_parameter('vehicle_length', 2.5)
        self.declare_parameter('dt', 0.02)  # 50 Hz
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        
        # Get parameters
        self.L = self.get_parameter('vehicle_length').value
        self.dt = self.get_parameter('dt').value
        
        # State: [x, y, theta, v, omega]
        self.state = np.array([
            self.get_parameter('initial_x').value,
            self.get_parameter('initial_y').value,
            self.get_parameter('initial_theta').value,
            0.0,  # v
            0.0   # omega
        ])
        
        # Control input: [v, omega]
        self.control = np.zeros(2)
        
        # Trajectory history
        self.trajectory = Path()
        self.trajectory.header.frame_id = 'map'
        
        # Simulation timer (50 Hz)
        self.sim_timer = self.create_timer(self.dt, self.simulation_step)
        
        # Obstacle timer (publish once)
        self.create_timer(1.0, self.publish_obstacles)
        
        # Track which command topic is being used
        self.use_cbf = False  # Set to True when CBF node is active
        self.cbf_active_time = None
        
        self.get_logger().info('Vehicle Simulator Node initialized')
        self.get_logger().info(f'Initial state: x={self.state[0]}, y={self.state[1]}, theta={self.state[2]}')
    
    def cmd_safe_callback(self, msg):
        """Receive safe control commands from CBF"""
        self.control[0] = msg.linear.x   # v
        self.control[1] = msg.angular.z  # omega
        self.use_cbf = True
        self.cbf_active_time = self.get_clock().now()
    
    def cmd_nom_callback(self, msg):
        """Receive nominal control commands from controller"""
        # Check if CBF is active (received message in last 200ms)
        if self.use_cbf and self.cbf_active_time is not None:
            time_since_cbf = (self.get_clock().now() - self.cbf_active_time).nanoseconds / 1e9
            if time_since_cbf < 0.2:
                # CBF is active, ignore nominal commands
                return
        
        # Use nominal commands
        self.control[0] = msg.linear.x   # v
        self.control[1] = msg.angular.z  # omega
    
    def simulation_step(self):
        """Simulate vehicle dynamics using bicycle model"""
        x, y, theta, v, omega = self.state
        v_cmd, omega_cmd = self.control
        
        # Bicycle model kinematics (ported from MATLAB simulate_step.m)
        # Compute actual velocities (with some dynamics, could add filtering)
        v_actual = v_cmd  # For simplicity, assuming instant response
        omega_actual = omega_cmd
        
        # Update state using Runge-Kutta 4th order (or simple Euler)
        # Simple Euler integration:
        x_dot = v_actual * cos(theta)
        y_dot = v_actual * sin(theta)
        theta_dot = omega_actual
        
        # Update state
        self.state[0] += x_dot * self.dt      # x
        self.state[1] += y_dot * self.dt      # y
        self.state[2] += theta_dot * self.dt  # theta
        self.state[3] = v_actual              # v
        self.state[4] = omega_actual          # omega
        
        # Normalize theta to [-pi, pi]
        self.state[2] = atan2(sin(self.state[2]), cos(self.state[2]))
        
        # Publish state
        self.publish_state()
        
        # Publish odometry
        self.publish_odometry()
        
        # Publish vehicle marker
        self.publish_vehicle_marker()
        
        # Publish TF transform
        self.publish_tf()
        
        # Update and publish trajectory
        self.update_trajectory()
    
    def publish_state(self):
        """Publish current state as PoseStamped"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.position.x = self.state[0]
        msg.pose.position.y = self.state[1]
        msg.pose.position.z = 0.0
        
        # Convert theta to quaternion
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = sin(self.state[2] / 2.0)
        msg.pose.orientation.w = cos(self.state[2] / 2.0)
        
        self.state_pub.publish(msg)
    
    def publish_odometry(self):
        """Publish odometry for robot_state_publisher"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        
        # Position
        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sin(self.state[2] / 2.0)
        msg.pose.pose.orientation.w = cos(self.state[2] / 2.0)
        
        # Velocity
        msg.twist.twist.linear.x = self.state[3]
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = self.state[4]
        
        self.odom_pub.publish(msg)
    
    def publish_vehicle_marker(self):
        """Publish vehicle as an arrow marker"""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'vehicle'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = self.state[0]
        marker.pose.position.y = self.state[1]
        marker.pose.position.z = 0.1
        
        # Orientation
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = sin(self.state[2] / 2.0)
        marker.pose.orientation.w = cos(self.state[2] / 2.0)
        
        # Scale (arrow size)
        marker.scale.x = 1.0  # Arrow length
        marker.scale.y = 0.2  # Arrow width
        marker.scale.z = 0.2  # Arrow height
        
        # Color (blue for vehicle)
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        
        self.vehicle_marker_pub.publish(marker)
    
    def publish_tf(self):
        """Publish TF transform for visualization"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sin(self.state[2] / 2.0)
        t.transform.rotation.w = cos(self.state[2] / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
    
    def update_trajectory(self):
        """Add current position to trajectory history"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.state[0]
        pose.pose.position.y = self.state[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = sin(self.state[2] / 2.0)
        pose.pose.orientation.w = cos(self.state[2] / 2.0)
        
        self.trajectory.poses.append(pose)
        self.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Publish trajectory (limit to last 1000 points)
        if len(self.trajectory.poses) > 1000:
            self.trajectory.poses.pop(0)
        
        self.traj_pub.publish(self.trajectory)
    
    def publish_obstacles(self):
        """Publish static obstacles (similar to MATLAB environment)"""
        marker_array = MarkerArray()
        
        # Example obstacles (matching MATLAB setup)
        obstacles = [
            {'x': 5.0, 'y': 5.0, 'radius': 1.0},
            {'x': -5.0, 'y': -5.0, 'radius': 1.0},
            {'x': 5.0, 'y': -5.0, 'radius': 1.5},
            {'x': -5.0, 'y': 5.0, 'radius': 1.5},
        ]
        
        for i, obs in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = obs['x']
            marker.pose.position.y = obs['y']
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = obs['radius'] * 2.0
            marker.scale.y = obs['radius'] * 2.0
            marker.scale.z = 1.0
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # Forever
            
            marker_array.markers.append(marker)
        
        self.obstacle_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(obstacles)} obstacles')


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
