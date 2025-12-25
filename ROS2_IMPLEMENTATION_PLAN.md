# ROS2 Implementation Plan

## 📋 Overview

Based on the PDF requirements, we need to create a **minimal but functional ROS2 package** that demonstrates the autonomous vehicle navigation system.

---

## 🎯 Requirements Summary (from PDF)

### Package Structure
```
vehicle_nav/
├── package.xml
├── setup.py
├── vehicle_nav/
│   ├── astar_node.py
│   ├── rrtstar_node.py
│   ├── pid_node.py
│   ├── lqr_node.py
│   ├── mpc_node.py
│   └── cbf_node.py
├── launch/
│   └── sim_launch.py
└── config/
    └── params.yaml
```

### Functional Requirements
1. **Each node must run and demonstrate core functionality**
2. **Planner nodes**: Accept current state → produce trajectory
3. **CBF node**: Filter control signal for safety
4. **Controller nodes**: Execute the trajectory

### Documentation Requirements
- Installation instructions
- ROS2 version specification
- How to run each simulation
- Example commands
- File manifest with I/O specifications

---

## 🗂️ Implementation Plan

### Phase 1: Workspace & Package Setup (30 min)
**Goal**: Create ROS2 workspace and package skeleton

**Tasks**:
- ✅ Create ROS2 workspace at `ros2_ws/`
- [ ] Create `vehicle_nav` package
- [ ] Setup `package.xml` with dependencies
- [ ] Setup `setup.py` with entry points
- [ ] Create folder structure (launch/, config/, vehicle_nav/)

**Dependencies to include**:
- `rclpy` (ROS2 Python client library)
- `std_msgs` (standard messages)
- `geometry_msgs` (Pose, Twist, etc.)
- `nav_msgs` (Path, Odometry)
- `sensor_msgs` (if needed)

---

### Phase 2: Message Definitions & Interfaces (20 min)
**Goal**: Define custom messages or use standard ROS2 messages

**Topics Architecture**:
```
/state          → Current vehicle state [geometry_msgs/PoseStamped]
/planned_path   → Path from planner [nav_msgs/Path]
/cmd_vel_nom    → Nominal control from controller [geometry_msgs/Twist]
/cmd_vel_safe   → Safe control after CBF [geometry_msgs/Twist]
/obstacles      → Environment obstacles [custom or PointCloud2]
```

**Tasks**:
- [ ] Define topic names in params.yaml
- [ ] Document message types
- [ ] Create simple state publisher (for testing)

---

### Phase 3: Planner Nodes (2 hours)
**Goal**: Implement A* and RRT* planner nodes

#### 3.1 A* Node (`astar_node.py`)
**Inputs**:
- `/state` → Current position
- `/goal` → Goal position
- `/obstacles` → Environment obstacles (static)

**Outputs**:
- `/planned_path` → Waypoint list

**Implementation**:
- [ ] Port MATLAB A* algorithm to Python
- [ ] ROS2 node wrapper with subscribers/publishers
- [ ] Grid-based search with Manhattan heuristic
- [ ] Path smoothing (optional)

#### 3.2 RRT* Node (`rrtstar_node.py`)
**Inputs**: Same as A*
**Outputs**: Same as A*

**Implementation**:
- [ ] Port MATLAB RRT* to Python
- [ ] Sampling-based algorithm
- [ ] Collision checking
- [ ] Path optimization

---

### Phase 4: Controller Nodes (2.5 hours)
**Goal**: Implement PID, LQR, MPC controllers

#### 4.1 PID Controller (`pid_node.py`)
**Inputs**:
- `/state` → Current state
- `/planned_path` → Reference trajectory

**Outputs**:
- `/cmd_vel_nom` → [v, ω] control

**Implementation**:
- [ ] Port MATLAB PID controller
- [ ] Path tracking logic
- [ ] Lookahead calculation
- [ ] Tuning parameters from config

#### 4.2 LQR Controller (`lqr_node.py`)
**Inputs**: Same as PID
**Outputs**: Same as PID

**Implementation**:
- [ ] Port MATLAB LQR controller
- [ ] State-space model
- [ ] Optimal gain calculation
- [ ] Reference tracking

#### 4.3 MPC Controller (`mpc_node.py`)
**Inputs**: Same as PID
**Outputs**: Same as PID

**Implementation**:
- [ ] Port MATLAB MPC controller
- [ ] Optimization solver (CVXPY or similar)
- [ ] Prediction horizon
- [ ] Constraint handling

---

### Phase 5: CBF Safety Node (1.5 hours)
**Goal**: Implement Control Barrier Function safety filter

#### CBF Node (`cbf_node.py`)
**Inputs**:
- `/state` → Current state
- `/cmd_vel_nom` → Nominal control
- `/obstacles` → Obstacle positions

**Outputs**:
- `/cmd_vel_safe` → Safety-filtered control
- `/cbf_active` → Boolean flag

**Implementation**:
- [ ] Port MATLAB CBF filter
- [ ] QP solver (CVXPY)
- [ ] Barrier function computation
- [ ] Safety constraint enforcement

---

### Phase 6: Launch & Configuration (1 hour)
**Goal**: Create launch files and parameter configuration

#### Launch File (`sim_launch.py`)
**Features**:
- [ ] Launch all nodes with parameters
- [ ] Configurable planner/controller selection
- [ ] Environment parameters
- [ ] Remapping topics if needed

#### Configuration (`params.yaml`)
**Parameters**:
- [ ] Vehicle parameters (wheelbase, max velocity, etc.)
- [ ] Environment (obstacles, bounds)
- [ ] Controller gains (PID, LQR, MPC)
- [ ] CBF parameters (safety margin, alpha)
- [ ] Planner settings (resolution, max iterations)

---

### Phase 7: Testing & Integration (2 hours)
**Goal**: Verify all nodes work together

**Tests**:
- [ ] Individual node testing
- [ ] Planner → Controller integration
- [ ] Controller → CBF integration
- [ ] Full pipeline: State → Plan → Control → CBF → Execute
- [ ] Multiple configurations (different planners/controllers)

---

### Phase 8: Documentation (1.5 hours)
**Goal**: Complete ROS2 documentation

#### README.md
- [ ] Installation instructions (ROS2 Humble/Iron)
- [ ] Package dependencies
- [ ] Build instructions (`colcon build`)
- [ ] How to run (`ros2 launch vehicle_nav sim_launch.py`)
- [ ] Node descriptions
- [ ] Topic interface documentation

#### FILE_MANIFEST.md
- [ ] List all Python files
- [ ] Purpose of each node
- [ ] Inputs/outputs for each node
- [ ] Message types

---

## 📊 Implementation Timeline

| Phase | Duration | Priority |
|-------|----------|----------|
| 1. Workspace Setup | 30 min | HIGH |
| 2. Message Definitions | 20 min | HIGH |
| 3. Planner Nodes | 2 hours | HIGH |
| 4. Controller Nodes | 2.5 hours | HIGH |
| 5. CBF Node | 1.5 hours | MEDIUM |
| 6. Launch & Config | 1 hour | MEDIUM |
| 7. Testing | 2 hours | HIGH |
| 8. Documentation | 1.5 hours | MEDIUM |
| **TOTAL** | **~11 hours** | |

---

## 🔧 Technical Approach

### Code Reuse Strategy
Since we have working MATLAB code, we will:
1. **Port algorithms** to Python (NumPy/SciPy)
2. **Wrap in ROS2 nodes** with minimal changes
3. **Use similar parameters** for consistency
4. **Validate** against MATLAB results

### Simplified Assumptions
To keep it "minimal but functional":
1. **2D navigation** (no elevation)
2. **Static obstacles** (pre-defined)
3. **Perfect state observation** (no estimator in ROS2)
4. **Simple dynamics** (bicycle model)
5. **Synchronous execution** (not real-time critical)

### Python Libraries Needed
- `numpy` - Numerical operations
- `scipy` - LQR computation, optimization
- `cvxpy` - QP solver (MPC, CBF)
- `matplotlib` - Visualization (optional)
- `rclpy` - ROS2 Python client

---

## 🚀 Getting Started

### Step 1: Create Package
```bash
cd ros2_ws/src
ros2 pkg create vehicle_nav --build-type ament_python --dependencies rclpy std_msgs geometry_msgs nav_msgs
```

### Step 2: Add Nodes
Copy template structure and implement each node following MATLAB logic.

### Step 3: Build & Test
```bash
cd ros2_ws
colcon build --packages-select vehicle_nav
source install/setup.bash
ros2 launch vehicle_nav sim_launch.py
```

---

## 📝 Success Criteria

✅ **Functional**:
- All 6 nodes run without errors
- Planners generate valid paths
- Controllers track paths
- CBF filters controls for safety

✅ **Integrated**:
- Nodes communicate via topics
- Launch file starts all nodes
- Parameters configurable via YAML

✅ **Documented**:
- README with clear instructions
- File manifest with I/O specs
- Example commands work

---

## 🎯 Next Steps

1. **Start with Phase 1**: Create package and folder structure
2. **Implement planners first**: They're independent and easier to test
3. **Then controllers**: Can test with dummy paths
4. **Add CBF last**: Requires both planners and controllers
5. **Integration testing**: Use launch file to test full pipeline

---

**Estimated completion time**: 1-2 days with focused work

**Ready to begin?** Let's start with Phase 1: Package setup!
