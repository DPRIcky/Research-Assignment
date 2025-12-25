# Testing Guide

## Quick Start

### Run Automated Tests

To test all 12 combinations automatically:

```bash
cd /home/prajjwal/Reseaarch\ Assignment\ 26\ Dec,\ 2025/ros2_ws
source install/setup.bash
bash src/vehicle_nav/scripts/run_tests.sh
```

This will:
- Test all 6 planner/controller combinations with and without CBF
- Collect performance metrics for each test
- Save results to `test_results/` directory
- Generate a summary report

⏱️ **Estimated time**: ~10 minutes (12 tests × 60s timeout each)

### Analyze Results

After tests complete:

```bash
python3 src/vehicle_nav/scripts/analyze_results.py test_results
```

This generates:
- Performance comparison table
- Best configuration rankings
- CBF impact analysis

## Manual Testing

### Test Single Configuration

```bash
# Terminal 1: Launch system
source install/setup.bash
ros2 launch vehicle_nav sim_launch.py planner:=astar controller:=lqr use_cbf:=true use_rviz:=true

# Terminal 2: Start metrics collector
ros2 run vehicle_nav metrics_node --ros-args -p test_name:=manual_test -p output_file:=manual_result.json

# Terminal 3: Publish goal
ros2 run vehicle_nav goal_publisher 8.0 8.0
```

### Test Without Metrics

Just launch and visualize:

```bash
source install/setup.bash
ros2 launch vehicle_nav sim_launch.py planner:=rrtstar controller:=mpc use_cbf:=true use_rviz:=true
ros2 run vehicle_nav goal_publisher 8.0 8.0
```

## Metrics Collected

For each test, the system records:
- **Goal Reached**: Success/failure
- **Execution Time**: Time to reach goal
- **Planning Time**: Time to compute first path
- **Path Length**: Total distance traveled
- **Tracking Error**: Mean/max deviation from planned path
- **Control Effort**: Average control magnitude
- **Safety Violations**: Obstacle collisions (should be 0 with CBF)

## Test Configurations

| Planner | Controller | CBF | Expected Behavior |
|---------|-----------|-----|-------------------|
| A* | PID | Yes/No | Fast planning, moderate tracking |
| A* | LQR | Yes/No | Fast planning, optimal tracking |
| A* | MPC | Yes/No | Fast planning, predictive control |
| RRT* | PID | Yes/No | Smooth paths, moderate tracking |
| RRT* | LQR | Yes/No | Smooth paths, optimal tracking |
| RRT* | MPC | Yes/No | Smooth paths, predictive control |

## Troubleshooting

**Tests timeout**: Increase timeout in `run_tests.sh` (line 9):
```bash
TIMEOUT=120  # Increase to 120 seconds
```

**Processes don't die**: Manually kill:
```bash
pkill -9 -f ros2
```

**Missing results**: Check if goal is reachable and timeout is sufficient

## Expected Results

Typical outcomes:
- **A* planners**: Faster planning (<0.1s), may have sharp turns
- **RRT* planners**: Slower planning (0.5-2s), smoother paths
- **PID controller**: Moderate tracking, simple control
- **LQR controller**: Better tracking, optimal gains
- **MPC controller**: Best tracking, higher computation
- **With CBF**: Slightly slower, zero safety violations
- **Without CBF**: Faster, possible near-misses
