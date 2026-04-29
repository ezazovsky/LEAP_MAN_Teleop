# Coordinate Frames in RealMan RM65 Teleoperation System

## Overview

The teleoperation system for the RealMan RM65 robotic arm uses **four primary coordinate frames** to enable real-time control via Vive tracker input. Data flows through a series of transformations, starting from tracker motion in the world frame and ending with joint commands to the robot.

---

## The Four Coordinate Frames

### 1. **SteamVR World Frame** (TrackingUniverseStanding)

**Origin:** Floor level at tracking space center  
**Convention:** Right-handed coordinate system  
**Type:** Absolute reference frame (stationary)

**Characteristics:**
- Defined by the Vive base stations (lighthouses)
- Origin is typically set at floor level in the tracked space
- Serves as the absolute reference for all Vive tracker devices
- Fixed during the entire teleoperation session
- All device tracking data from OpenVR is expressed in this frame

**Data Format:**
- 4×4 homogeneous transformation matrix (SE(3))
- Retrieved via `tracker.get_T()` from OpenVR API
- Structure:
  ```
  T = [ R  | t ]    where R is 3×3 rotation matrix
      [ 0  | 1 ]          t is 3×1 position vector (meters)
  ```

**Relationship to Robot:**
- Separated from robot base by fixed offset: `base_station_origin = [3.0, -2.8, -3.0]` meters
- This represents the physical position of the Vive base station relative to the robot center

---

### 2. **Vive Tracker Frame** (End-Effector Input Device)

**Origin:** Center of physical Vive tracker device  
**Convention:** Moves with the hand holding the tracker  
**Type:** Mobile/dynamic reference frame

**Characteristics:**
- Represents the hand/controller manipulator in tracking space
- Continuously updated at ~125 Hz (control frequency)
- Position and orientation change as the operator moves their hand
- Expressed in the SteamVR world frame (4×4 matrix)
- Rotations tracked via 3×3 rotation matrix component

**Data Format:**
- 4×4 homogeneous transformation matrix obtained from OpenVR
- Position: `T[:3, 3]` → (x, y, z) in meters
- Rotation: `T[:3, :3]` → 3×3 rotation matrix

**Coordinate System:**
- Default tracker orientation at rest defines the local axes
- All motion is measured relative to this frame's current state
- Relative deltas computed at each control cycle: `T_delta = inv(tracker_home_T) @ current_T`

**Role in Teleoperation:**
- Primary input sensor for commanding end-effector motion
- Incremental motion from home position drives the robot
- Must be transformed to robot frame via axis remapping

---

### 3. **Robot Base Frame** (RM65 Mechanical Reference)

**Origin:** Base of RM65 arm structure  
**Convention:** Z-axis pointing up, X-axis forward (at zero configuration)  
**Type:** Fixed reference frame attached to robot

**Characteristics:**
- Fundamental frame for robot kinematics
- Defined by RM65 hardware mechanical design
- Remains fixed relative to the physical arm
- All forward kinematics solutions expressed in this frame
- Workspace is bounded by safety limits (see below)

**Workspace Constraints:**
- **Cartesian bounds:**
  - X: [-0.37, 0.37] meters (±37 cm left-right)
  - Y: [-0.37, 0.37] meters (±37 cm front-back)
  - Z: [0.05, 0.40] meters (5 cm off table, 40 cm max height)
- **Reach radius (cylindrical constraint):**
  - Hard limit: 0.58 meters (avoid singularities)
  - Soft damping region: > 0.54 meters
  - Minimum reach: 0.15 meters (avoid self-collision)

**Data Format:**
- Cartesian pose: 6-element vector `[x, y, z, rx, ry, rz]`
  - x, y, z: position in meters
  - rx, ry, rz: orientation as Euler angles in radians (ZYX convention)
- Obtained via forward kinematics: `rm_algo_forward_kinematics(joint_angles, 1)`

**Relationship to Tracker Frame:**
- Requires axis remapping to convert tracker motion to robot motion
- Remapping rule (Vive frame → Robot frame):
  ```
  Robot_X = -Vive_Y    (swap and negate)
  Robot_Y = -Vive_X    (swap and negate)
  Robot_Z = -Vive_Z    (negate only)
  ```
- Same transformation applied to rotation vectors

---

### 4. **End-Effector Frame** (Tool/Gripper Reference)

**Origin:** Distal end of RM65 arm (gripper/tool mount point)  
**Convention:** Inherits from robot base frame  
**Type:** Mobile end frame (moves with arm)

**Characteristics:**
- Attached to the end of the robotic arm
- Moves with all arm joint motion
- Position and orientation determined by forward kinematics
- Target frame for inverse kinematics solving
- Where the LEAP Hand gripper is physically mounted

**Data Format:**
- Cartesian pose: `[x, y, z, rx, ry, rz]` in robot base frame
- Obtained from forward kinematics of current joint angles
- Commanded via inverse kinematics: `rm_movep_canfd(pose, follow, mode, radio)`

**Spatial Relationship:**
- Distance from robot base varies: 0.15 m to 0.58 m
- Always within the workspace constraints defined above
- Position tracks the tracker motion after:
  1. Axis remapping from tracker frame
  2. Safety bounds enforcement
  3. Smoothing interpolation

---

## Frame Transformation Pipeline

### Complete Data Flow Diagram

```
SteamVR World Frame
    ↓
[1] tracker.get_T() → T_absolute [4×4]
    ↓
[2] Apply base_station_origin offset
    T_centered = origin_inv @ T_absolute
    ↓
Robot-Centered Frame (workspace coordinate system)
    ↓
[3] Calibration: store tracker_home_T and robot_home_pose
    ↓
[4] Each control cycle: compute relative delta
    T_delta = inv(tracker_home_T) @ T_centered
    ↓
[5] AXIS REMAP: Vive frame → Robot frame
    pos_delta: [x, y, z] → [-y, -x, -z]
    rot_delta: [rx, ry, rz] → [-ry, -rx, -rz]
    ↓
[6] Add delta to home pose: raw_pose = home + remapped_delta
    ↓
[7] Safety filtering (6-step process):
    • Jump protection (glitch filter)
    • Cartesian box clamp
    • Reach radius clamp (spherical constraint)
    • Singularity avoidance
    ↓
[8] Smoothing interpolation (EMA position + SLERP rotation)
    ↓
Robot Base Frame - Safe Smoothed Pose
    ↓
[9] Inverse kinematics: pose → joint angles
    ↓
RM65 Controller Joint Commands
    ↓
End-Effector Frame Motion
```

---

## Transformations Between Frames

### Transformation 1: SteamVR World → Robot-Centered Frame

**Operation:** Apply base station offset
```
centered_pose = origin_inv @ world_pose
```

**Purpose:** 
- Accounts for the physical placement of the Vive base station relative to the robot
- Transforms absolute world coordinates to displacement from robot center
- Fixed transformation applied once per tracking cycle

**Inverse Transformation:**
```
world_pose = base_station_origin @ centered_pose
```

---

### Transformation 2: Tracker Frame → Relative Motion (Delta)

**Operation:** Compute difference from calibration point
```
T_delta = inv(tracker_home_T) @ T_current
```

**Purpose:**
- Computes incremental motion from the home/calibration position
- Ignores absolute position in world frame; focuses on user hand motion relative to starting position
- Enables intuitive teleoperation: small hand movements → small arm movements

**Extraction:**
```
pos_delta = T_delta[:3, 3]           # Position difference vector
R_delta = T_delta[:3, :3]             # Rotation matrix difference
rotvec_delta = R.from_matrix(R_delta).as_rotvec()  # Convert to axis-angle
```

---

### Transformation 3: Axis Remapping (Critical Frame Conversion)

**Operation:**
```python
# Position remapping
remapped_pos = [
    -pos_delta[1],       # Vive-Y → Robot-X (negate and swap)
    -pos_delta[0],       # Vive-X → Robot-Y (negate and swap)
    -pos_delta[2]        # Vive-Z → Robot-Z (negate only)
]

# Rotation remapping (apply same transformation)
remapped_rotvec = [
    -rotvec_delta[1],    # Same axis swap
    -rotvec_delta[0],
    -rotvec_delta[2]
]

# Convert back to Euler angles
euler_delta = R.from_rotvec(remapped_rotvec).as_euler('xyz', degrees=False)
```

**Purpose:**
- Aligns Vive tracker coordinate system with RM65 robot coordinate system
- Handles physical orientation of Vive base station relative to robot workspace
- Makes teleoperation intuitive despite different frame orientations
- Negation accounts for "mirrored" reference frames

**Why This Mapping?**
- Vive base station is positioned offset from robot center (3.0, -2.8, -3.0)
- This offset and orientation defines the swap/negate pattern
- Empirically calibrated during system setup

---

### Transformation 4: Home-Relative Pose Computation

**Operation:**
```
target_pose = robot_home_pose + remapped_delta
```

**Components:**
- `robot_home_pose`: [x_h, y_h, z_h, rx_h, ry_h, rz_h] from calibration
- `remapped_delta`: Transformed tracker motion
- `target_pose`: Desired end-effector position command

**Format:**
```
target_pose[0:3] = home_position + remapped_pos_delta
target_pose[3:6] = home_rotation + euler_delta
```

---

### Transformation 5: Safety Bounds Enforcement (Multi-Step Filtering)

**Step 1: Jump Protection**
- Detect glitches: if position jump > 0.1 m, limit to max_pos_jump
- Only move toward commanded point at rates respecting arm capability limits
- Short-path angle filtering for rotations

**Step 2: Cartesian Box Clamp**
```python
safe_pose[0] = clamp(pose[0], -0.37, 0.37)   # X bounds
safe_pose[1] = clamp(pose[1], -0.37, 0.37)   # Y bounds
safe_pose[2] = clamp(pose[2],  0.05, 0.40)   # Z bounds
```

**Step 3: Reach Radius Clamp (Cylindrical Constraint)**
```
r = sqrt(x² + y² + z²)

if r > soft_radius (0.54 m):
    Apply gentle damping toward center
    
if r > hard_radius (0.58 m):
    Scale position to exactly hard_radius
    
if r < min_radius (0.15 m):
    Prevent approaching base too closely
```

**Step 4 (Optional): Singularity Avoidance**
- Detects shoulder singularity zone (joint angle configuration)
- Prevents further outward motion in critical zones
- Biases motion slightly inward

**Result:** `safe_pose` guaranteed to be within all bounds

---

### Transformation 6: Smoothing (Exponential Moving Average + SLERP)

**Purpose:**
- Reduces jitter from tracker noise
- Creates smooth motion for arm following
- Prevents jerky acceleration/deceleration

**Position Smoothing (EMA):**
```
T = (1 - decay_factor)  # Typically 0.15 for 125 Hz control
smoothed_pos = (decay_factor) * last_smoothed_pos + 
               (1 - decay_factor) * current_safe_pos
```

**Rotation Smoothing (SLERP):**
```
q_smooth = SLERP(q_last, q_current, t=1-decay_factor)
              # Spherical Linear Interpolation
```

**Output:** `smoothed_pose` ready for inverse kinematics

---

### Transformation 7: Inverse Kinematics (IK)

**Operation:** Convert Cartesian end-effector pose to joint angles
```
joint_angles = rm_algo_inverse_kinematics(smoothed_pose)
```

**Command to Robot:**
```
status = rm_movep_canfd(
    smoothed_pose,      # Target: [x, y, z, rx, ry, rz]
    follow_mode=True,   # Continuous update mode
    speed_mode=1,
    radio=0             # Communication parameter
)
```

**Result:** RM65 joint motors move to achieve the end-effector pose

---

## 3D Frame Visualization

Below is a 3D representation of the coordinate frame relationships in the teleoperation workspace:

```
                    ↑ Z (World Up)
                    |
    SteamVR World Frame (Absolute Reference)
    ├─ Origin at floor level in tracked space
    └─ Defined by lighthouse base stations
                    |
      ┌─────────────┼─────────────┐
      │             │             │
    (X)           origin         (Y)
   (Right)        [3.0, -2.8, -3.0]
                    │
                    ↓
    Robot-Centered Frame
    ├─ Offset applied: origin_inv @ T_tracker
    └─ Workspace center
                    │
       ┌────────────┼────────────┐
       │            │            │
     Vive         Robot         End
    Tracker       Base         Effector
    Frame         Frame         Frame
       │            │            │
    ┌──┴──┐     ┌────┴────┐   ┌──┴──┐
    │Hand │     │ Box     │   │Tool │
    │Move │     │Constraint    │Move │
    │Sensor     │±37cm XY      │Actuator
    └─────┘     │0.05-0.4cm Z  └─────┘
                │Radius        
                │0.15-0.58m    
                └─────────────┘
```

### Interactive 3D Diagram

```
        World Frame (SteamVR)
         ┏━━━━━━━━━━━━┓
         ┃   Y-AXIS   ┃
         ┃  ^ + +     ┃         Vive Tracker Frame
         ┃ /|  ╱╱╱    ┃        ┌─────────────────┐
         ┃/ |╱╱ ╱     ┃        │   (Mobile)      │
         ┃  └╱╱───>Z  ┃        │  ┌─────────┐    │
         ┃    X (out) ┃        │  │ Tracker │    │
         ┗━━━━━━━━━━━━┛        │  └─────────┘    │
              │                │   Moves with    │
              │  base_station  │   operator hand │
              │  origin offset │                 │
              │  [3.0,-2.8,-3.0]                 │
              │  meters        │  Axis Remap:    │
              ↓                │  X←-Y  Y←-X  Z←-Z
           ┌────────────────┐  │               
           │ Robot-Centered │  └─────────────────┘
           │   Workspace    │
           └────────────────┘
              │      │      │
         ┌────┴──┬───┴──┬───┴────┐
         │       │      │        │
    ┌────┴─┐ ┌──┴───┐  │   ┌──┴──┐
    │ Soft │ │ Hard │  │   │ End │
    │Reach │ │Reach │  │   │Eff. │
    │0.54m │ │0.58m │  │   │ Fk │
    └──────┘ └──────┘  │   └─────┘
                       │
                   ┌───┴────┐
                   │ Box    │
                   │Bounds  │
                   │±37cmXY │
                   │0.05-0.4│
                   └────────┘
```

---

## Real-Time Data Flow Summary

| Stage | Input | Operation | Output | Frame |
|-------|-------|-----------|--------|-------|
| 1 | Vive absolute | `get_T()` | 4×4 matrix | World → Robot-Centered |
| 2 | Relative delta | `inv(home_T) @ current` | 4×4 ΔT | Tracker → Relative Motion |
| 3 | Relative pose | Axis remap [-y,-x,-z] | 6-element [x,y,z,rx,ry,rz] | Vive → Robot coordinates |
| 4 | Remapped delta | Add to robot_home_pose | 6-element target | Relative → Absolute in robot frame |
| 5 | Raw target | 6-step safety filter | 6-element safe_pose | Cartesian enforcement |
| 6 | Safe pose | EMA + SLERP smooth | 6-element smoothed_pose | Jitter reduction |
| 7 | Smoothed pose | `rm_movep_canfd()` | Joint commands | IK solving |
| 8 | Joint angles | Forward kinematics | Cartesian EE pose | Robot base → EE frame |

---

## Calibration: Anchoring the Frames

**Calibration Process** (occurs once at start):

```python
# User holds Vive tracker steady, system waits 3 seconds
tracker_home_T = get_current_tracker_matrix()  # 4×4 matrix
robot_home_pose = get_current_robot_pose()     # [x,y,z,rx,ry,rz] via FK

# Verify home is within all safety bounds
assert x in [-0.37, 0.37]
assert y in [-0.37, 0.37]
assert z in [0.05, 0.40]
assert radius in [0.15, 0.58]
```

**Purpose:**
- Establishes the correspondence between tracker space and robot space
- Determines the "zero point" for incremental motion commands
- All subsequent motion is relative to this calibration point
- Ensures arm starts in safe configuration

---

## Key Invariants & Safety Properties

1. **Position invariant:** End-effector stays within workspace bounds
2. **Continuity invariant:** No teleportation; motion limited by max velocity
3. **Frame consistency:** All position commands expressed in robot base frame
4. **Rotation smoothness:** Shortest-path angle interpolation prevents gimbal lock
5. **Axis correspondence:** Vive hand motion maps intuitively to arm motion after remapping

---

## Integration with System Components

- **Vive Tracker (Input):** Provides T_world via OpenVR @ 125 Hz
- **Teleoperation Controller:** Performs all transformations, filtering, and smoothing
- **RM65 Robot (Output):** Interprets Cartesian poses via its internal IK solver
- **LEAP Hand (Output):** Receives separate hand pose from MANUS glove (not frame-dependent)
- **Logger:** Records raw_pose, safe_pose, and smoothed_pose at each control cycle

---

## Summary

The teleoperation system uses a well-defined chain of coordinate frame transformations to convert hand tracker motion into safe, smooth end-effector commands. The four primary frames are:

1. **SteamVR World Frame** — Absolute reference
2. **Vive Tracker Frame** — Hand input sensor  
3. **Robot Base Frame** — Mechanical reference with workspace bounds
4. **End-Effector Frame** — Tool output on arm

The transformation from tracker to robot involves axis remapping, safety filtering, and smoothing interpolation, culminating in Cartesian pose commands sent to the RM65 arm.
