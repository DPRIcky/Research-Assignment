# Vehicle Navigation System - ROS2

Autonomous vehicle navigation system with multiple path planning algorithms, controllers, and safety filtering.

## Features

- **2 Path Planners**: A* (grid-based), RRT* (sampling-based)
- **3 Controllers**: PID, LQR, MPC
- **Safety Filter**: Control Barrier Functions (CBF) with obstacle avoidance
- **Visualization**: RViz2 integration
- **Configurable**: Easy parameter tuning via YAML

## System Architecture

```
Goal → Planner → Path → Controller → Nominal Control → CBF → Safe Control → Simulator → State
                                                                                ↓
                                                                            Obstacles
```

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/goal` | PoseStamped | Target goal position |
| `/state` | PoseStamped | Current vehicle state |
| `/planned_path` | Path | Path from planner |
| `/cmd_vel_nom` | Twist | Nominal control from controller |
| `/cmd_vel_safe` | Twist | CBF-filtered safe control |
| `/obstacles` | MarkerArray | Environment obstacles |
| `/vehicle_marker` | Marker | Vehicle visualization |
| `/trajectory_history` | Path | Actual path taken |
| `/odom` | Odometry | Odometry data |

## Installation

### Prerequisites

- ROS2 Humble
- Python 3.10+
- Dependencies: numpy, scipy

### Build

```bash
cd ros2_ws
colcon build --packages-select vehicle_nav
source install/setup.bash
```

## Usage

### Launch System

```bash
# Basic launch with A* planner and PID controller
ros2 launch vehicle_nav sim_launch.py

# With specific configuration
ros2 launch vehicle_nav sim_launch.py planner:=rrtstar controller:=lqr use_cbf:=true use_rviz:=true
```

### Launch Arguments

| Argument | Options | Default | Description |
|----------|---------|---------|-------------|
| `planner` | `astar`, `rrtstar` | `astar` | Path planning algorithm |
| `controller` | `pid`, `lqr`, `mpc` | `pid` | Control algorithm |
| `use_cbf` | `true`, `false` | `false` | Enable CBF safety filter |
| `use_rviz` | `true`, `false` | `false` | Launch RViz visualization |

### Publish Goal

```bash
# Publish goal at (x, y)
ros2 run vehicle_nav goal_publisher 8.0 8.0
```

### Example Configurations

```bash
# Fast planning with optimal control and safety
ros2 launch vehicle_nav sim_launch.py planner:=astar controller:=lqr use_cbf:=true

# Smooth paths with predictive control
ros2 launch vehicle_nav sim_launch.py planner:=rrtstar controller:=mpc use_cbf:=true

# Simple reactive control
ros2 launch vehicle_nav sim_launch.py planner:=astar controller:=pid use_cbf:=false
```

## Nodes

### Simulator Node

Simulates vehicle dynamics using bicycle kinematics model.

**Subscriptions:**
- `/cmd_vel_safe` (Twist) - Uses when CBF active
- `/cmd_vel_nom` (Twist) - Fallback when CBF inactive

**Publications:**
- `/state` (PoseStamped) - 50 Hz
- `/odom` (Odometry) - Vehicle odometry
- `/obstacles` (MarkerArray) - Environment obstacles
- `/vehicle_marker` (Marker) - Vehicle visualization
- `/trajectory_history` (Path) - Actual trajectory

**Parameters:**
- `vehicle.wheelbase`: 0.5m
- `vehicle.max_speed`: 5.0 m/s
- `vehicle.max_angular_velocity`: π/2 rad/s
- `environment.bounds`: [-10, 10]

### A* Planner Node

Grid-based path planning with optimal guarantees on grid.

**Subscriptions:**
- `/state` (PoseStamped)
- `/goal` (PoseStamped)
- `/obstacles` (MarkerArray)

**Publications:**
- `/planned_path` (Path)

**Parameters:**
- `astar.grid_resolution`: 0.1m
- `environment.bounds`: [-10, 10]

**Features:**
- Manhattan distance heuristic
- 8-connected neighbors
- Path orientation computation
- Fast replanning on new goals

### RRT* Planner Node

Sampling-based path planning with asymptotic optimality.

**Subscriptions:**
- `/state` (PoseStamped)
- `/goal` (PoseStamped)
- `/obstacles` (MarkerArray)

**Publications:**
- `/planned_path` (Path)

**Parameters:**
- `rrtstar.max_iterations`: 3000
- `rrtstar.step_size`: 2.0m
- `rrtstar.search_radius`: 5.0m
- `rrtstar.goal_sample_rate`: 0.1
- `rrtstar.safety_margin`: 1.5m

**Features:**
- Goal-biased sampling
- Rewiring for optimality
- Collision checking with safety margin
- Smooth, near-optimal paths

### PID Controller Node

Reactive control with pure pursuit path tracking.

**Subscriptions:**
- `/state` (PoseStamped)
- `/planned_path` (Path)

**Publications:**
- `/cmd_vel_nom` (Twist)

**Parameters:**
- `pid.kp_v`: 1.0 (linear velocity P gain)
- `pid.ki_v`: 0.1 (linear velocity I gain)
- `pid.kd_v`: 0.05 (linear velocity D gain)
- `pid.kp_w`: 2.0 (angular velocity P gain)
- `pid.ki_w`: 0.2 (angular velocity I gain)
- `pid.kd_w`: 0.1 (angular velocity D gain)
- `pid.lookahead_distance`: 1.0m

**Features:**
- Pure pursuit lookahead
- Anti-windup protection
- Separate PID for linear/angular
- Local frame error transformation

### LQR Controller Node

Optimal state feedback control with discrete-time algebraic Riccati equation.

**Subscriptions:**
- `/state` (PoseStamped)
- `/planned_path` (Path)

**Publications:**
- `/cmd_vel_nom` (Twist)

**Parameters:**
- `lqr.Q`: [1.0, 1.0, 2.0] (state cost weights)
- `lqr.R`: [0.1, 0.1] (control cost weights)
- `lqr.lookahead_distance`: 2.0m

**Features:**
- System linearization around reference
- DARE solver for optimal gains
- Minimal control effort
- Fast computation

### MPC Controller Node

Model Predictive Control with finite horizon optimization.

**Subscriptions:**
- `/state` (PoseStamped)
- `/planned_path` (Path)

**Publications:**
- `/cmd_vel_nom` (Twist)

**Parameters:**
- `mpc.horizon`: 10 (prediction steps)
- `mpc.Q`: [1.0, 1.0, 2.0] (state cost)
- `mpc.R`: [0.1, 0.1] (control cost)
- `mpc.Qf`: [10.0, 10.0, 20.0] (terminal cost)
- `mpc.lookahead_distance`: 2.0m

**Features:**
- Warm starting with shifted solution
- Terminal cost for stability
- Constraint handling
- SLSQP optimizer

### CBF Safety Filter Node

Control Barrier Function-based safety filter with active obstacle avoidance.

**Subscriptions:**
- `/cmd_vel_nom` (Twist)
- `/state` (PoseStamped)
- `/obstacles` (MarkerArray)

**Publications:**
- `/cmd_vel_safe` (Twist)

**Parameters:**
- `cbf.safety_margin`: 0.3m
- `cbf.alpha`: 0.5 (class-K function parameter)
- `cbf.repulsive_gain`: 3.0 (obstacle avoidance strength)
- `cbf.repulsive_distance`: 3.0m (activation range)

**Features:**
- QP-based minimal intervention
- Active repulsive steering
- Lie derivative constraints
- Hard safety guarantees

## Configuration

Edit `config/params.yaml` to tune parameters:

```yaml
/**:
  ros__parameters:
    # Vehicle parameters
    vehicle:
      wheelbase: 0.5
      max_speed: 5.0
      max_angular_velocity: 1.57
    
    # Environment
    environment:
      bounds: [-10.0, 10.0]
    
    # Planner parameters
    astar:
      grid_resolution: 0.1
    
    rrtstar:
      max_iterations: 3000
      step_size: 2.0
      search_radius: 5.0
      goal_sample_rate: 0.1
      safety_margin: 1.5
    
    # Controller parameters
    pid:
      kp_v: 1.0
      ki_v: 0.1
      kd_v: 0.05
      kp_w: 2.0
      ki_w: 0.2
      kd_w: 0.1
      lookahead_distance: 1.0
    
    lqr:
      Q: [1.0, 1.0, 2.0]
      R: [0.1, 0.1]
      lookahead_distance: 2.0
    
    mpc:
      horizon: 10
      Q: [1.0, 1.0, 2.0]
      R: [0.1, 0.1]
      Qf: [10.0, 10.0, 20.0]
      lookahead_distance: 2.0
    
    # CBF parameters
    cbf:
      safety_margin: 0.3
      alpha: 0.5
      repulsive_gain: 3.0
      repulsive_distance: 3.0
```

## RViz Setup

1. Launch with RViz:
   ```bash
   ros2 launch vehicle_nav sim_launch.py use_rviz:=true
   ```

2. Manually add displays if needed:
   - **Path** → `/planned_path`
   - **Path** → `/trajectory_history`
   - **Marker** → `/vehicle_marker`
   - **MarkerArray** → `/obstacles`
   - Set Fixed Frame to `map`

3. Save configuration to `config/vehicle_nav.rviz`

## Troubleshooting

### Vehicle doesn't move
- Check if goal is published: `ros2 topic echo /goal`
- Verify path is generated: `ros2 topic echo /planned_path`
- Check controller output: `ros2 topic echo /cmd_vel_nom`

### Path goes through obstacles
- Increase planner safety margin
- Enable CBF: `use_cbf:=true`
- Check obstacle radius in simulator

### Poor tracking performance
- Tune controller gains in `params.yaml`
- Increase lookahead distance
- Try different controller (MPC usually best)

### CBF stops vehicle
- Already fixed with repulsive control
- Adjust `cbf.repulsive_gain` if too aggressive
- Increase `cbf.repulsive_distance` for earlier steering

### RViz displays not showing
- Check Fixed Frame is `map`
- Verify topics are publishing: `ros2 topic list`
- Manually add displays (auto-config may not work)

## Algorithm Comparison

| Algorithm | Speed | Path Quality | Tracking | Complexity |
|-----------|-------|--------------|----------|------------|
| **A* + PID** | Fast | Moderate | Moderate | Low |
| **A* + LQR** | Fast | Moderate | Good | Medium |
| **A* + MPC** | Fast | Moderate | Best | High |
| **RRT* + PID** | Slow | Best | Moderate | Low |
| **RRT* + LQR** | Slow | Best | Good | Medium |
| **RRT* + MPC** | Slow | Best | Best | High |

### Recommendations

- **Fast response needed**: A* + LQR
- **Best path quality**: RRT* + MPC
- **Computational constraints**: A* + PID
- **Safety critical**: Any + CBF
- **General purpose**: A* + LQR + CBF

## Development

### Adding New Planner

1. Create `new_planner_node.py` in `vehicle_nav/`
2. Subscribe to `/state`, `/goal`, `/obstacles`
3. Publish to `/planned_path`
4. Add entry point in `setup.py`
5. Add to `sim_launch.py`

### Adding New Controller

1. Create `new_controller_node.py` in `vehicle_nav/`
2. Subscribe to `/state`, `/planned_path`
3. Publish to `/cmd_vel_nom`
4. Add entry point in `setup.py`
5. Add to `sim_launch.py`

## License

MIT

## References

- A* Algorithm: Hart, P. E., Nilsson, N. J., & Raphael, B. (1968)
- RRT*: Karaman, S., & Frazzoli, E. (2011)
- LQR: Anderson, B. D., & Moore, J. B. (1990)
- MPC: Camacho, E. F., & Alba, C. B. (2013)
- CBF: Ames, A. D., et al. (2016)
