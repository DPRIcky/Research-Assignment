# ROS2 Navigation System - File Manifest

## Directory Structure

```
ros2_ws/src/vehicle_nav/
├── vehicle_nav/               # Python package
│   ├── __init__.py
│   ├── simulator_node.py      # Vehicle simulator
│   ├── astar_node.py          # A* planner
│   ├── rrtstar_node.py        # RRT* planner
│   ├── pid_node.py            # PID controller
│   ├── lqr_node.py            # LQR controller
│   ├── mpc_node.py            # MPC controller
│   ├── cbf_node.py            # CBF safety filter
│   ├── goal_publisher.py      # Goal publishing utility
│   ├── metrics_node.py        # Metrics collection
│   └── turtlesim_bridge.py    # TurtleSim integration (unused)
├── launch/
│   └── sim_launch.py          # Main launch file
├── config/
│   ├── params.yaml            # ROS2 parameters
│   └── vehicle_nav.rviz       # RViz configuration
├── scripts/
│   ├── run_tests.sh           # Automated testing
│   └── analyze_results.py     # Results analysis
├── resource/
│   └── vehicle_nav            # Package marker
├── package.xml                # Package manifest
├── setup.py                   # Python setup
└── README.md                  # Documentation
```

## Node Details

### 1. simulator_node.py (~310 lines)

**Purpose**: Simulate vehicle dynamics and environment

**Key Functions:**
- `timer_callback()`: Main simulation loop (50 Hz)
- `bicycle_model()`: Vehicle kinematics
- `publish_obstacles()`: Static obstacle publishing
- `publish_markers()`: Visualization

**Subscriptions:**
- `/cmd_vel_safe` → Uses when CBF active
- `/cmd_vel_nom` → Fallback

**Publications:**
- `/state`, `/odom`, `/vehicle_marker`, `/obstacles`, `/trajectory_history`

### 2. astar_node.py (~295 lines)

**Purpose**: Grid-based optimal path planning

**Key Functions:**
- `plan()`: A* search algorithm
- `heuristic()`: Manhattan distance
- `get_neighbors()`: 8-connected grid
- `is_collision()`: Obstacle checking
- `publish_path()`: Path with orientations

**Algorithm**: Complete, optimal on grid, deterministic

### 3. rrtstar_node.py (~380 lines)

**Purpose**: Sampling-based near-optimal planning

**Key Functions:**
- `plan()`: RRT* main loop
- `sample()`: Random/goal-biased sampling
- `nearest()`: Find closest node
- `steer()`: Extend toward sample
- `rewire()`: Optimize local tree
- `extract_path()`: Backtrack solution

**Algorithm**: Probabilistically complete, asymptotically optimal

### 4. pid_node.py (~270 lines)

**Purpose**: Reactive path tracking control

**Key Functions:**
- `control_callback()`: Main control loop (50 Hz)
- `pure_pursuit()`: Lookahead point selection
- `compute_control()`: PID computation
- `transform_error()`: Local frame transformation

**Features**: Separate linear/angular PID, anti-windup

### 5. lqr_node.py (~280 lines)

**Purpose**: Optimal state feedback control

**Key Functions:**
- `control_callback()`: Main control loop (50 Hz)
- `compute_lqr_control()`: Optimal gains from DARE
- `get_reference()`: Lookahead reference point
- `linearize_system()`: System matrices A, B

**Features**: Continuous DARE solving, minimal effort

### 6. mpc_node.py (~295 lines)

**Purpose**: Predictive control with constraints

**Key Functions:**
- `control_callback()`: Main control loop (50 Hz)
- `compute_mpc_control()`: Horizon optimization
- `mpc_cost()`: Quadratic cost function
- `predict_state()`: Forward simulation

**Features**: Warm starting, terminal penalty, SLSQP

### 7. cbf_node.py (~340 lines)

**Purpose**: Safety filtering and obstacle avoidance

**Key Functions:**
- `cmd_nom_callback()`: Filter nominal control
- `filter_control()`: QP solver for safety
- `get_cbf_constraints()`: Barrier constraints
- `compute_barrier()`: h(x) and gradients
- `compute_repulsive_control()`: Active steering

**Features**: Two-layer safety (repulsive + CBF), minimal intervention

### 8. goal_publisher.py (~40 lines)

**Purpose**: CLI utility to publish goals

**Usage**: `ros2 run vehicle_nav goal_publisher 8.0 8.0`

### 9. metrics_node.py (~280 lines)

**Purpose**: Performance metrics collection

**Key Functions:**
- `compute_tracking_error()`: Distance to path
- `check_collision()`: Safety violations
- `save_metrics()`: JSON output

**Metrics**: Success rate, time, path length, tracking error, safety

## Launch System

### sim_launch.py (~176 lines)

**Launch Arguments:**
- `planner`: astar | rrtstar
- `controller`: pid | lqr | mpc
- `use_cbf`: true | false
- `use_rviz`: true | false

**Conditional Node Launching:**
```python
DeclareLaunchArgument('planner', default_value='astar')
PythonExpression(["'", planner, "' == 'astar'"])
```

**Nodes Launched:**
1. simulator_node (always)
2. planner_node (conditional)
3. controller_node (conditional)
4. cbf_node (if use_cbf)
5. rviz2 (if use_rviz)

## Configuration Files

### params.yaml

ROS2 parameter format with global namespace `/**:`

**Sections:**
- Vehicle: wheelbase, max speeds
- Environment: bounds, obstacles
- Planners: A*, RRT* settings
- Controllers: PID, LQR, MPC gains
- CBF: safety margins, repulsive gains

### vehicle_nav.rviz

RViz2 display configuration (auto-generated, may need manual setup)

**Displays:**
- Path displays (planned, trajectory)
- Marker displays (vehicle, obstacles)
- Grid, axes

## Testing Scripts

### run_tests.sh

Bash script for automated testing

**Process:**
1. Loop through all combinations
2. Launch system
3. Start metrics node
4. Publish goal
5. Wait for completion
6. Save results
7. Clean up

**Output**: `test_results/*.json`, `summary_*.txt`

### analyze_results.py

Python script for result analysis

**Features:**
- Load all JSON results
- Group by configuration
- Compute statistics
- Generate comparison tables
- Identify best performers
- CBF impact analysis

## Dependencies

### ROS2 Packages
- `rclpy`: ROS2 Python client
- `std_msgs`: Standard messages
- `geometry_msgs`: Pose, Twist, Transform
- `nav_msgs`: Path, Odometry
- `sensor_msgs`: LaserScan (unused)
- `visualization_msgs`: Marker, MarkerArray
- `tf2_ros`: Transform broadcasting

### Python Packages
- `numpy`: Numerical computations
- `scipy`: Optimization (DARE, minimize)
- `math`: Basic math functions
- `json`: Metrics serialization
- `datetime`: Timestamps

## Message Types Used

| Type | Topics | Fields Used |
|------|--------|-------------|
| PoseStamped | /state, /goal | position.{x,y}, orientation |
| Path | /planned_path, /trajectory_history | poses[] |
| Twist | /cmd_vel_nom, /cmd_vel_safe | linear.x, angular.z |
| MarkerArray | /obstacles | markers[] |
| Marker | /vehicle_marker | pose, scale, color |
| Odometry | /odom | pose, twist |

## Coordinate Frames

- **map**: Global fixed frame, origin at environment center
- **base_footprint**: Vehicle frame, origin at rear axle
- Transform: `map → base_footprint` (published by simulator)

## Parameters Summary

### Critical Parameters to Tune

**Performance:**
- `pid.kp_v`, `pid.kp_w`: Responsiveness
- `lqr.Q`, `lqr.R`: State vs control tradeoff
- `mpc.horizon`: Prediction length

**Safety:**
- `cbf.safety_margin`: Clearance from obstacles
- `cbf.repulsive_gain`: Steering strength
- `rrtstar.safety_margin`: Planning clearance

**Planning:**
- `astar.grid_resolution`: Grid fineness
- `rrtstar.max_iterations`: Planning time
- `rrtstar.search_radius`: Optimality range

## Common Issues & Solutions

| Issue | File | Solution |
|-------|------|----------|
| No path orientation | astar_node.py | Compute from waypoint direction |
| Vehicle through obstacle | rrtstar_node.py | Add safety margin to collision check |
| CBF stops vehicle | cbf_node.py | Add repulsive control |
| RViz displays missing | vehicle_nav.rviz | Manual configuration needed |
| High tracking error | params.yaml | Increase lookahead distance |

## Build System

- **Package type**: ament_python
- **Build tool**: colcon
- **Entry points**: Defined in setup.py
- **Data files**: Launch, config copied to install/share/

## Version History

- v1.0.0: Initial implementation
  - All planners, controllers, CBF
  - Testing framework
  - Documentation
