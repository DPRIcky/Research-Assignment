# Quick Reference Guide

## One-Page Cheat Sheet

### Launch Commands

```bash
# Source workspace first
cd /home/prajjwal/Reseaarch\ Assignment\ 26\ Dec,\ 2025/ros2_ws
source install/setup.bash

# Basic launch
ros2 launch vehicle_nav sim_launch.py

# Full launch with visualization and safety
ros2 launch vehicle_nav sim_launch.py planner:=rrtstar controller:=mpc use_cbf:=true use_rviz:=true

# Publish goal
ros2 run vehicle_nav goal_publisher 8.0 8.0
```

### Configuration Matrix

| Planner | Controller | CBF | Best For |
|---------|-----------|-----|----------|
| astar | pid | false | Quick testing |
| astar | lqr | true | Fast & safe |
| astar | mpc | true | Balanced |
| rrtstar | lqr | true | Smooth paths |
| rrtstar | mpc | true | Best overall |
| rrtstar | pid | false | Development |

### Topic Monitoring

```bash
# Check state
ros2 topic echo /state

# Check path
ros2 topic echo /planned_path

# Check control
ros2 topic echo /cmd_vel_nom

# Check safe control (if CBF enabled)
ros2 topic echo /cmd_vel_safe

# List all topics
ros2 topic list

# Check rates
ros2 topic hz /state
```

### Parameter Quick Edits

Edit `config/params.yaml`:

```yaml
# Make vehicle faster
vehicle.max_speed: 7.0  # default: 5.0

# Make PID more responsive
pid.kp_w: 3.0  # default: 2.0

# Make CBF less aggressive
cbf.repulsive_gain: 2.0  # default: 3.0

# Make RRT* faster (less optimal)
rrtstar.max_iterations: 1500  # default: 3000

# Make A* finer resolution
astar.grid_resolution: 0.05  # default: 0.1
```

Then rebuild:
```bash
colcon build --packages-select vehicle_nav
source install/setup.bash
```

### Common Debugging

```bash
# No path generated?
ros2 topic echo /goal  # Check goal received
ros2 node list  # Check all nodes running

# Vehicle not moving?
ros2 topic echo /cmd_vel_nom  # Check controller output
ros2 topic echo /planned_path  # Check path exists

# RViz issues?
# Manually add displays:
# Add → By Topic → /planned_path → Path
# Add → By Topic → /obstacles → MarkerArray
# Set Fixed Frame to "map"

# CBF too aggressive?
# Edit params.yaml:
cbf.repulsive_gain: 2.0
cbf.safety_margin: 0.2

# Kill all ROS2 processes
pkill -9 -f ros2
```

### File Locations

```
Config:          config/params.yaml
Launch:          launch/sim_launch.py
RViz:            config/vehicle_nav.rviz
Nodes:           vehicle_nav/*.py
Documentation:   README.md, FILE_MANIFEST.md
```

### Algorithm Selection Guide

**Choose A*** when:
- Need deterministic results
- Fast planning required
- Grid-based environment
- Computational constraints

**Choose RRT*** when:
- Need smooth paths
- Complex environments
- Path quality > speed
- Avoiding sharp turns

**Choose PID** when:
- Simple reactive control needed
- Development/testing
- Low computation available

**Choose LQR** when:
- Optimal control desired
- Fast computation needed
- Good tracking required

**Choose MPC** when:
- Best tracking needed
- Can afford computation
- Constraint handling important

**Enable CBF** when:
- Safety is critical
- Obstacles are close
- Hard guarantees needed
- In production use

### Performance Expectations

| Configuration | Plan Time | Track Error | Safety |
|---------------|-----------|-------------|--------|
| A*/PID | <0.1s | 0.3-0.5m | Moderate |
| A*/LQR | <0.1s | 0.1-0.3m | Good |
| A*/MPC | <0.1s | 0.05-0.15m | Good |
| RRT*/PID | 0.5-2s | 0.3-0.5m | Moderate |
| RRT*/LQR | 0.5-2s | 0.1-0.3m | Good |
| RRT*/MPC | 0.5-2s | 0.05-0.15m | Best |
| +CBF | +0-10% | +0-0.05m | Guaranteed |

### Typical Workflow

1. **Development/Testing**:
   ```bash
   ros2 launch vehicle_nav sim_launch.py planner:=astar controller:=pid use_rviz:=true
   ```

2. **Tuning Parameters**:
   - Edit `config/params.yaml`
   - Rebuild: `colcon build --packages-select vehicle_nav`
   - Source: `source install/setup.bash`
   - Test changes

3. **Final Validation**:
   ```bash
   ros2 launch vehicle_nav sim_launch.py planner:=rrtstar controller:=mpc use_cbf:=true
   ```

4. **Multiple Goals**:
   ```bash
   # In separate terminal after launch
   ros2 run vehicle_nav goal_publisher 5.0 5.0
   # Wait for completion
   ros2 run vehicle_nav goal_publisher -5.0 -5.0
   # And so on...
   ```

### Troubleshooting Flowchart

```
Issue: Nothing happens
└─> Check: ros2 node list
    └─> Nodes missing? 
        └─> Check launch succeeded
    └─> Nodes running?
        └─> Check: ros2 topic echo /goal
            └─> No goal?
                └─> Publish: ros2 run vehicle_nav goal_publisher X Y

Issue: Path not followed
└─> Check: ros2 topic echo /cmd_vel_nom
    └─> Zero output?
        └─> Check path exists: ros2 topic echo /planned_path
    └─> Non-zero but not moving?
        └─> Check CBF: ros2 topic echo /cmd_vel_safe
            └─> Zeros? → CBF too aggressive
                └─> Reduce cbf.repulsive_gain

Issue: Goes through obstacles
└─> Enable CBF: use_cbf:=true
└─> Or increase: rrtstar.safety_margin

Issue: Too slow
└─> Use astar: planner:=astar
└─> Reduce: rrtstar.max_iterations
└─> Increase: vehicle.max_speed
```

### Environment Layout

Default obstacle positions (radius 1.0-1.5m):
- Obstacle 1: (5, 5)
- Obstacle 2: (-5, -5)
- Obstacle 3: (5, -5)
- Obstacle 4: (-5, 5)

Good test goals:
- `8.0 8.0` - Diagonal through gap
- `-8.0 8.0` - Across top
- `0.0 0.0` - Return to start
- `8.0 -8.0` - Different diagonal

### Key Takeaways

✓ Always source workspace before running
✓ RViz may need manual display configuration
✓ CBF adds safety with minimal overhead
✓ MPC gives best tracking, PID is simplest
✓ RRT* paths are smoother than A*
✓ Tune parameters in params.yaml, then rebuild
✓ Use `pkill -9 -f ros2` to reset everything
