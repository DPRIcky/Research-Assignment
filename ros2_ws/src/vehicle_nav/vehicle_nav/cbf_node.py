#!/usr/bin/env python3
"""
CBF Safety Filter Node
Subscribes to /cmd_vel_nom (nominal control) and /obstacles
Publishes to /cmd_vel_safe (safe control)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import MarkerArray
import numpy as np
import math
from scipy.optimize import minimize

class CBFNode(Node):
    def __init__(self):
        super().__init__('cbf_filter')
        
        # Publishers
        self.cmd_safe_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        
        # Subscribers
        self.create_subscription(Twist, '/cmd_vel_nom', self.cmd_nom_callback, 10)
        self.create_subscription(PoseStamped, '/state', self.state_callback, 10)
        self.create_subscription(MarkerArray, '/obstacles', self.obstacles_callback, 10)
        
        # Parameters
        self.declare_parameter('cbf.safety_margin', 0.3)  # Reduced from 1.5 to allow passage
        self.declare_parameter('cbf.alpha', 0.5)  # Reduced for less aggressive filtering
        self.declare_parameter('cbf.repulsive_gain', 3.0)  # Gain for obstacle avoidance steering
        self.declare_parameter('cbf.repulsive_distance', 3.0)  # Distance threshold for repulsion
        self.declare_parameter('vehicle.max_speed', 5.0)
        self.declare_parameter('vehicle.max_angular_velocity', math.pi / 2)
        
        # Get parameters
        self.safety_margin = self.get_parameter('cbf.safety_margin').value
        self.alpha = self.get_parameter('cbf.alpha').value
        self.repulsive_gain = self.get_parameter('cbf.repulsive_gain').value
        self.repulsive_distance = self.get_parameter('cbf.repulsive_distance').value
        self.v_max = self.get_parameter('vehicle.max_speed').value
        self.w_max = self.get_parameter('vehicle.max_angular_velocity').value
        
        # State variables
        self.current_state = None
        self.obstacles = []
        self.nominal_cmd = None
        
        # Statistics
        self.total_commands = 0
        self.modified_commands = 0
        
        self.get_logger().info(f'CBF Safety Filter initialized (margin={self.safety_margin}m, alpha={self.alpha}, repulsive_gain={self.repulsive_gain})')
    
    def state_callback(self, msg):
        """Store current state"""
        self.current_state = msg
    
    def obstacles_callback(self, msg):
        """Update obstacles"""
        self.obstacles = []
        for marker in msg.markers:
            obs = {
                'type': 'circle',
                'x': marker.pose.position.x,
                'y': marker.pose.position.y,
                'radius': marker.scale.x / 2.0
            }
            self.obstacles.append(obs)
    
    def cmd_nom_callback(self, msg):
        """Receive nominal control and apply CBF filter"""
        if self.current_state is None:
            self.get_logger().warn('No state received, passing through nominal control', throttle_duration_sec=2.0)
            self.cmd_safe_pub.publish(msg)
            return
        
        # Extract state
        x = self.current_state.pose.position.x
        y = self.current_state.pose.position.y
        q = self.current_state.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        state = np.array([x, y, theta])
        
        # Nominal control
        v_nom = msg.linear.x
        w_nom = msg.angular.z
        
        # Add repulsive control to steer away from obstacles
        w_repulsive = self.compute_repulsive_control(state)
        v_augmented = v_nom
        w_augmented = w_nom + w_repulsive
        
        # Apply CBF filter to augmented control
        v_safe, w_safe, is_modified = self.filter_control(state, v_augmented, w_augmented)
        
        # Statistics
        self.total_commands += 1
        if is_modified:
            self.modified_commands += 1
        
        # Log when control is modified
        if is_modified or abs(w_repulsive) > 0.01:
            self.get_logger().info(
                f'Control: nom=({v_nom:.2f}, {w_nom:.2f}) repulsive_w={w_repulsive:.2f} safe=({v_safe:.2f}, {w_safe:.2f})',
                throttle_duration_sec=0.5
            )
        
        # Publish safe control
        cmd_safe = Twist()
        cmd_safe.linear.x = v_safe
        cmd_safe.angular.z = w_safe
        self.cmd_safe_pub.publish(cmd_safe)
    
    def filter_control(self, state, v_nom, w_nom):
        """
        Apply CBF safety filter
        Returns: (v_safe, w_safe, is_modified)
        """
        # Get CBF constraints
        A_cbf, b_cbf = self.get_cbf_constraints(state)
        
        if len(A_cbf) == 0:
            # No obstacles nearby, use nominal control
            return v_nom, w_nom, False
        
        # QP formulation: min ||u - u_nom||^2 subject to CBF constraints
        # Decision variable: u = [v, w]
        u_nom = np.array([v_nom, w_nom])
        
        # Bounds on controls
        bounds = [(0.0, self.v_max), (-self.w_max, self.w_max)]
        
        # Solve QP using scipy.optimize.minimize with SLSQP
        def objective(u):
            return np.sum((u - u_nom)**2)
        
        def constraint_func(u):
            # CBF constraints: A*u <= b --> -A*u + b >= 0
            return b_cbf - A_cbf @ u
        
        constraints = {'type': 'ineq', 'fun': constraint_func}
        
        try:
            result = minimize(
                objective,
                u_nom,
                method='SLSQP',
                bounds=bounds,
                constraints=constraints,
                options={'maxiter': 100, 'ftol': 1e-4}  # Increased iterations, relaxed tolerance
            )
            
            if result.success:
                v_safe = float(result.x[0])
                w_safe = float(result.x[1])
                
                # Check if control was modified
                is_modified = (abs(v_safe - v_nom) > 1e-3) or (abs(w_safe - w_nom) > 1e-3)
                
                return v_safe, w_safe, is_modified
            else:
                # QP failed - try to find ANY feasible solution
                # Check if we can at least stop safely
                test_constraint = constraint_func(np.array([0.0, 0.0]))
                if np.all(test_constraint >= -1e-3):  # Stopping is feasible
                    self.get_logger().warn('CBF QP failed, stopping vehicle', throttle_duration_sec=1.0)
                    return 0.0, 0.0, True
                else:
                    # Even stopping violates constraints - allow nominal control
                    self.get_logger().warn('CBF constraints infeasible, using nominal control', throttle_duration_sec=1.0)
                    return v_nom, w_nom, False
                
        except Exception as e:
            self.get_logger().warn(f'CBF optimization error: {e}', throttle_duration_sec=1.0)
            return v_nom, w_nom, False  # Use nominal on error
    
    def get_cbf_constraints(self, state):
        """
        Compute CBF constraints for differential drive
        Returns: A, b such that A*u <= b where u = [v, w]
        """
        x, y, theta = state
        
        A_list = []
        b_list = []
        
        # Check each obstacle
        for obs in self.obstacles:
            # Compute barrier function and gradient
            h, dh_dx, dh_dy = self.compute_barrier(x, y, obs)
            
            # Skip if obstacle is far away
            if h > 10.0:
                continue
            
            # For differential drive with control u = [v; w]:
            # dx/dt = v*cos(theta)
            # dy/dt = v*sin(theta)
            
            # Lie derivative with respect to control
            # dh/dt = (dh/dx)*(dx/dt) + (dh/dy)*(dy/dt)
            #       = (dh/dx)*v*cos(theta) + (dh/dy)*v*sin(theta)
            #       = [dh/dx*cos(theta) + dh/dy*sin(theta)] * v + 0 * w
            
            Lg_h = np.array([
                dh_dx * math.cos(theta) + dh_dy * math.sin(theta),
                0.0
            ])
            
            # CBF constraint: Lg_h*u + alpha*h >= 0
            # Rearranged: -Lg_h*u <= alpha*h
            # Use max to prevent overly restrictive constraints when h is small
            h_safe = max(h, -0.5)  # Don't let barrier get too negative
            A_list.append(-Lg_h)
            b_list.append(self.alpha * h_safe)
        
        if len(A_list) > 0:
            A = np.array(A_list)
            b = np.array(b_list)
        else:
            A = np.empty((0, 2))
            b = np.empty(0)
        
        return A, b
    
    def compute_barrier(self, x, y, obstacle):
        """
        Compute barrier function value and gradient
        h > 0 means safe, h = 0 means on boundary, h < 0 means collision
        Returns: (h, dh_dx, dh_dy)
        """
        if obstacle['type'] == 'circle':
            # Distance to circle center
            dx = x - obstacle['x']
            dy = y - obstacle['y']
            dist = math.sqrt(dx**2 + dy**2)
            
            # Barrier function: h = dist - (radius + safety_margin)
            h = dist - (obstacle['radius'] + self.safety_margin)
            
            # Gradient
            if dist > 1e-6:
                dh_dx = dx / dist
                dh_dy = dy / dist
            else:
                # At center, use arbitrary direction
                dh_dx = 1.0
                dh_dy = 0.0
            
            return h, dh_dx, dh_dy
        else:
            # Unknown obstacle type, assume safe
            return 1.0, 0.0, 0.0
    
    def compute_repulsive_control(self, state):
        """
        Compute repulsive angular velocity to steer away from nearby obstacles
        Returns: w_repulsive (angular velocity to add to nominal control)
        """
        x, y, theta = state
        w_repulsive = 0.0
        
        for obs in self.obstacles:
            # Vector from robot to obstacle
            dx_obs = obs['x'] - x
            dy_obs = obs['y'] - y
            dist = math.sqrt(dx_obs**2 + dy_obs**2)
            
            # Only apply repulsion if obstacle is within threshold distance
            if dist > self.repulsive_distance:
                continue
            
            # Angle to obstacle in global frame
            angle_to_obs = math.atan2(dy_obs, dx_obs)
            
            # Angle difference between robot heading and obstacle direction
            # Normalize to [-pi, pi]
            angle_diff = angle_to_obs - theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            
            # Repulsive force magnitude (inverse square law with saturation)
            repulsive_radius = obs['radius'] + self.safety_margin
            safe_dist = max(dist - repulsive_radius, 0.1)  # Prevent division by zero
            magnitude = self.repulsive_gain / (safe_dist**2)
            
            # Steering direction: turn away from obstacle
            # If obstacle is to the right (angle_diff > 0), turn left (w < 0)
            # If obstacle is to the left (angle_diff < 0), turn right (w > 0)
            if abs(angle_diff) < math.pi / 2:  # Obstacle is in front
                # Steer away based on which side obstacle is on
                w_repulsive -= magnitude * math.sin(angle_diff)
            
        # Saturate to reasonable limits
        w_repulsive = max(min(w_repulsive, self.w_max), -self.w_max)
        
        return w_repulsive
    
    def check_safety(self, state):
        """Check if current state is safe (no collisions)"""
        x, y = state[0], state[1]
        
        for obs in self.obstacles:
            h, _, _ = self.compute_barrier(x, y, obs)
            if h < 0:
                return False
        
        return True
    
    def get_statistics(self):
        """Get CBF statistics"""
        if self.total_commands > 0:
            modification_rate = 100.0 * self.modified_commands / self.total_commands
        else:
            modification_rate = 0.0
        
        return {
            'total_commands': self.total_commands,
            'modified_commands': self.modified_commands,
            'modification_rate': modification_rate
        }

def main(args=None):
    rclpy.init(args=args)
    node = CBFNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Print statistics on shutdown
        stats = node.get_statistics()
        node.get_logger().info(
            f"CBF Statistics: {stats['modified_commands']}/{stats['total_commands']} "
            f"commands modified ({stats['modification_rate']:.1f}%)"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
