# Topic Architecture and Message Definitions

## Overview
This document defines the complete topic architecture and message types for the vehicle_nav ROS2 package.

## Topic Graph

```
┌─────────────┐
│   Planner   │ (A* or RRT*)
│   Nodes     │
└──────┬──────┘
       │ subscribes: /goal (PoseStamped)
       │ publishes:  /planned_path (Path)
       │
       ▼
┌─────────────┐
│ Controller  │ (PID, LQR, or MPC)
│   Nodes     │
└──────┬──────┘
       │ subscribes: /state (PoseStamped)
       │            /planned_path (Path)
       │ publishes:  /cmd_vel_nom (Twist)
       │
       ▼
┌─────────────┐
│ CBF Safety  │
│   Filter    │
└──────┬──────┘
       │ subscribes: /cmd_vel_nom (Twist)
       │            /state (PoseStamped)
       │            /obstacles (MarkerArray)
       │ publishes:  /cmd_vel_safe (Twist)
       │
       ▼
   [Vehicle/Simulator]
```

## Topic Definitions

### 1. /state
**Message Type:** `geometry_msgs/msg/PoseStamped`

**Publisher:** Simulation/Vehicle state publisher (external)

**Subscribers:** All controller nodes, CBF safety filter

**Description:** Current vehicle state (position and orientation)

**Message Structure:**
```yaml
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id: "map"
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x        # Vehicle x position (m)
    float64 y        # Vehicle y position (m)
    float64 z        # Always 0.0 (2D navigation)
  geometry_msgs/Quaternion orientation
    float64 x, y, z  # Computed from theta
    float64 w        # Computed from theta
```

**Rate:** 50 Hz (matches control loop)

---

### 2. /goal
**Message Type:** `geometry_msgs/msg/PoseStamped`

**Publisher:** User interface / Mission planner (external)

**Subscribers:** Planner nodes (A*, RRT*)

**Description:** Goal position for path planning

**Message Structure:**
```yaml
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id: "map"
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x        # Goal x position (m)
    float64 y        # Goal y position (m)
    float64 z        # Always 0.0
  geometry_msgs/Quaternion orientation
    float64 x, y, z, w  # Goal orientation (optional)
```

**Rate:** Event-driven (published when new goal is set)

---

### 3. /planned_path
**Message Type:** `nav_msgs/msg/Path`

**Publishers:** Planner nodes (A*, RRT*)

**Subscribers:** All controller nodes

**Description:** Planned path from current position to goal

**Message Structure:**
```yaml
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id: "map"
geometry_msgs/PoseStamped[] poses  # Array of waypoints
  std_msgs/Header header
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x, y, z
    geometry_msgs/Quaternion orientation
      float64 x, y, z, w
```

**Rate:** Event-driven (published when path is computed)

**Notes:** 
- Path includes sequence of waypoints from start to goal
- Controllers interpolate between waypoints for smooth tracking

---

### 4. /cmd_vel_nom
**Message Type:** `geometry_msgs/msg/Twist`

**Publishers:** Controller nodes (PID, LQR, MPC)

**Subscribers:** CBF safety filter

**Description:** Nominal (desired) control commands before safety filtering

**Message Structure:**
```yaml
geometry_msgs/Vector3 linear
  float64 x        # Linear velocity (m/s)
  float64 y        # Always 0.0 (non-holonomic)
  float64 z        # Always 0.0 (2D navigation)
geometry_msgs/Vector3 angular
  float64 x        # Always 0.0 (2D navigation)
  float64 y        # Always 0.0 (2D navigation)
  float64 z        # Angular velocity (rad/s)
```

**Rate:** 50 Hz (control loop frequency)

**Constraints:**
- Linear velocity: [0, vehicle.max_speed] from params.yaml
- Angular velocity: [-vehicle.max_omega, vehicle.max_omega] from params.yaml

---

### 5. /cmd_vel_safe
**Message Type:** `geometry_msgs/msg/Twist`

**Publisher:** CBF safety filter

**Subscribers:** Vehicle/Simulator actuators

**Description:** Safety-filtered control commands (guaranteed collision-free)

**Message Structure:**
```yaml
geometry_msgs/Vector3 linear
  float64 x        # Safe linear velocity (m/s)
  float64 y        # Always 0.0
  float64 z        # Always 0.0
geometry_msgs/Vector3 angular
  float64 x        # Always 0.0
  float64 y        # Always 0.0
  float64 z        # Safe angular velocity (rad/s)
```

**Rate:** 50 Hz

**Notes:**
- If CBF is disabled, this equals /cmd_vel_nom
- CBF may reduce velocities to maintain safety constraints

---

### 6. /obstacles
**Message Type:** `visualization_msgs/msg/MarkerArray`

**Publisher:** Environment/Obstacle manager (external)

**Subscribers:** CBF safety filter

**Description:** Dynamic obstacle positions and geometries

**Message Structure:**
```yaml
visualization_msgs/Marker[] markers
  std_msgs/Header header
  string ns                    # Namespace
  int32 id                     # Unique obstacle ID
  int32 type                   # SPHERE, CYLINDER, CUBE, etc.
  int32 action                 # ADD, DELETE, MODIFY
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x, y, z          # Obstacle center
    geometry_msgs/Quaternion orientation
  geometry_msgs/Vector3 scale
    float64 x, y, z            # Obstacle dimensions
  std_msgs/ColorRGBA color
  duration lifetime
```

**Rate:** 10 Hz (or event-driven for static obstacles)

**Notes:**
- Used by CBF to compute safety constraints
- Can represent both static and dynamic obstacles

---

## QoS Policies

### State and Control Topics
- **Reliability:** `RELIABLE` (ensure delivery)
- **Durability:** `VOLATILE` (only current state matters)
- **History:** `KEEP_LAST(1)` (only latest message needed)
- **Deadline:** `20ms` (50 Hz control loop)

### Path Planning Topics
- **Reliability:** `RELIABLE`
- **Durability:** `TRANSIENT_LOCAL` (late subscribers receive last path)
- **History:** `KEEP_LAST(1)`

### Obstacle Topics
- **Reliability:** `RELIABLE`
- **Durability:** `TRANSIENT_LOCAL` (ensure all obstacles known)
- **History:** `KEEP_LAST(10)` (maintain recent obstacle history)

## Parameter Configuration

All topic names are configurable in [config/params.yaml](config/params.yaml):

```yaml
topics:
  state: '/state'
  goal: '/goal'
  planned_path: '/planned_path'
  cmd_vel_nom: '/cmd_vel_nom'
  cmd_vel_safe: '/cmd_vel_safe'
  obstacles: '/obstacles'
```

## Data Flow Summary

1. **Planning Phase:**
   - User publishes goal → `/goal`
   - Planner receives goal, computes path → `/planned_path`

2. **Control Phase:**
   - Controller subscribes to `/state` and `/planned_path`
   - Computes tracking control → `/cmd_vel_nom`

3. **Safety Filtering Phase:**
   - CBF receives `/cmd_vel_nom`, `/state`, `/obstacles`
   - Solves QP to find safe control → `/cmd_vel_safe`

4. **Execution:**
   - Vehicle/Simulator executes `/cmd_vel_safe`
   - Updates `/state` (closes the loop)

## Coordinate Frame

All topics use **"map"** frame:
- Origin: (0, 0) at environment center
- X-axis: Points right (East)
- Y-axis: Points up (North)
- Orientation: CCW from X-axis (standard ROS convention)

## Message Type Summary

| Topic | Message Type | Package |
|-------|-------------|---------|
| /state | PoseStamped | geometry_msgs |
| /goal | PoseStamped | geometry_msgs |
| /planned_path | Path | nav_msgs |
| /cmd_vel_nom | Twist | geometry_msgs |
| /cmd_vel_safe | Twist | geometry_msgs |
| /obstacles | MarkerArray | visualization_msgs |

All message types are standard ROS2 messages - no custom definitions required.
