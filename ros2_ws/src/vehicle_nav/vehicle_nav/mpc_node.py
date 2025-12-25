#!/usr/bin/env python3
"""
MPC Controller Node for trajectory tracking
Subscribes to /planned_path and /state
Publishes to /cmd_vel_nom
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import numpy as np
import math
from scipy.optimize import minimize

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_nom', 10)
        
        # Subscribers
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.create_subscription(PoseStamped, '/state', self.state_callback, 10)
        
        # Parameters
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('mpc.horizon', 10)
        self.declare_parameter('mpc.q_x', 1.0)
        self.declare_parameter('mpc.q_y', 1.0)
        self.declare_parameter('mpc.q_theta', 2.0)
        self.declare_parameter('mpc.r_v', 0.1)
        self.declare_parameter('mpc.r_w', 0.1)
        self.declare_parameter('mpc.qf_x', 10.0)
        self.declare_parameter('mpc.qf_y', 10.0)
        self.declare_parameter('mpc.qf_theta', 20.0)
        self.declare_parameter('vehicle.max_speed', 5.0)
        self.declare_parameter('vehicle.max_angular_velocity', math.pi / 2)
        
        # Get parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.N = self.get_parameter('mpc.horizon').value
        self.q_x = self.get_parameter('mpc.q_x').value
        self.q_y = self.get_parameter('mpc.q_y').value
        self.q_theta = self.get_parameter('mpc.q_theta').value
        self.r_v = self.get_parameter('mpc.r_v').value
        self.r_w = self.get_parameter('mpc.r_w').value
        self.qf_x = self.get_parameter('mpc.qf_x').value
        self.qf_y = self.get_parameter('mpc.qf_y').value
        self.qf_theta = self.get_parameter('mpc.qf_theta').value
        self.v_max = self.get_parameter('vehicle.max_speed').value
        self.w_max = self.get_parameter('vehicle.max_angular_velocity').value
        
        # Cost matrices
        self.Q = np.diag([self.q_x, self.q_y, self.q_theta])
        self.R = np.diag([self.r_v, self.r_w])
        self.Qf = np.diag([self.qf_x, self.qf_y, self.qf_theta])
        
        # State variables
        self.current_state = None
        self.planned_path = None
        
        # Control timer
        self.dt = 1.0 / self.control_rate
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # Warm start for optimization
        self.u_prev = None
        
        self.get_logger().info(f'MPC Controller Node initialized (N={self.N}, dt={self.dt:.3f})')
    
    def path_callback(self, msg):
        """Store the planned path"""
        self.planned_path = msg
        # Reset warm start when new path received
        self.u_prev = None
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
    
    def predict_state(self, x, v, w):
        """Predict next state using differential drive model"""
        theta = x[2]
        x_next = np.array([
            x[0] + v * math.cos(theta) * self.dt,
            x[1] + v * math.sin(theta) * self.dt,
            x[2] + w * self.dt
        ])
        return x_next
    
    def mpc_cost(self, u, x0, ref_traj):
        """
        Compute MPC cost function
        u: control sequence [v(0), w(0), ..., v(N-1), w(N-1)]
        x0: initial state [x, y, theta]
        ref_traj: reference trajectory (N+1 x 3)
        """
        cost = 0.0
        x = x0.copy()
        
        for k in range(self.N):
            # Extract control at step k
            v = u[2*k]
            w = u[2*k + 1]
            
            # Predict next state
            x = self.predict_state(x, v, w)
            
            # State error
            x_ref = ref_traj[k+1]
            x_err = np.array([
                x[0] - x_ref[0],
                x[1] - x_ref[1],
                self.normalize_angle(x[2] - x_ref[2])
            ])
            
            # Add to cost
            if k < self.N - 1:
                # Stage cost
                cost += x_err @ self.Q @ x_err + np.array([v, w]) @ self.R @ np.array([v, w])
            else:
                # Terminal cost
                cost += x_err @ self.Qf @ x_err + np.array([v, w]) @ self.R @ np.array([v, w])
        
        return cost
    
    def get_reference_trajectory(self, current_pos, path_poses):
        """Extract reference trajectory for MPC horizon"""
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
        
        # Get N+1 points for reference trajectory
        ref_traj = []
        for i in range(self.N + 1):
            idx = min(closest_idx + i, len(path_poses) - 1)
            pose = path_poses[idx]
            ref_traj.append([
                pose.pose.position.x,
                pose.pose.position.y,
                self.quaternion_to_yaw(pose.pose.orientation)
            ])
        
        return np.array(ref_traj), closest_idx
    
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
        state = np.array([x, y, theta])
        
        # Get reference trajectory
        ref_traj, closest_idx = self.get_reference_trajectory((x, y), self.planned_path.poses)
        
        # Check if reached goal
        goal_dist = math.sqrt(
            (ref_traj[-1][0] - x)**2 + (ref_traj[-1][1] - y)**2
        )
        if closest_idx == len(self.planned_path.poses) - 1 and goal_dist < 0.15:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info('Goal reached!', throttle_duration_sec=2.0)
            return
        
        # Initial guess for optimization (warm start if available)
        if self.u_prev is not None and len(self.u_prev) == 2 * self.N:
            # Shift previous solution and add small forward motion
            u0 = np.concatenate([self.u_prev[2:], [1.0, 0.0]])
        else:
            # Default: small forward velocity, zero angular
            u0 = np.tile([1.0, 0.0], self.N)
        
        # Bounds on controls: [v_min, v_max] and [-w_max, w_max]
        bounds = []
        for _ in range(self.N):
            bounds.append((0.0, self.v_max))  # v bounds
            bounds.append((-self.w_max, self.w_max))  # w bounds
        
        # Solve MPC optimization
        try:
            result = minimize(
                lambda u: self.mpc_cost(u, state, ref_traj),
                u0,
                method='SLSQP',
                bounds=bounds,
                options={'maxiter': 50, 'ftol': 1e-4}
            )
            u_opt = result.x
            self.u_prev = u_opt  # Store for warm start
        except Exception as e:
            self.get_logger().warn(f'MPC optimization failed: {e}')
            u_opt = u0
        
        # Extract first control
        v_cmd = float(u_opt[0])
        w_cmd = float(u_opt[1])
        
        # Apply constraints
        v_cmd = max(min(v_cmd, self.v_max), 0.0)
        w_cmd = max(min(w_cmd, self.w_max), -self.w_max)
        
        # Log for debugging
        self.get_logger().info(
            f'MPC Control: v={v_cmd:.2f}, w={w_cmd:.2f}, ref_idx={closest_idx}/{len(self.planned_path.poses)}',
            throttle_duration_sec=1.0
        )
        
        # Publish command
        cmd = Twist()
        cmd.linear.x = v_cmd
        cmd.angular.z = w_cmd
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
