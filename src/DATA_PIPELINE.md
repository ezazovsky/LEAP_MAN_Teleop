# Data Pipeline Architecture

Complete walkthrough of how data flows through the bimanual teleoperation system for recording and replay.

---

## Overview: Two Pipelines

### Pipeline 1: Live Teleoperation + Logging (teleop_recorder.py)
Real-time control with HDF5 recording of arm, hand, and synchronized RGB camera data.

### Pipeline 2: Replay from Recording (replay_hdf5.py)
Read HDF5 → Smooth homing → Replay trajectory at original timing.

---

## Pipeline 1: Live Teleoperation + Logging

### Stage 1A: Arm Data Acquisition

**Source:** Vive Tracker (openvr library)

```
ViveTrackerModule.get_T() 
  → 4×4 homogeneous transform matrix (SE(3))
  → Position [x, y, z] in meters
  → Rotation as 3×3 matrix
```

**Location:** `teleop_recorder.py`, line 354
```python
current_T = self.mapper.get_current_tracker_matrix()
```

**Data format:** `numpy.ndarray`, shape `(4, 4)`, dtype `float64`

---

### Stage 1B: Arm Pose Computation (Raw)

**Process:** Transform tracker matrix into Cartesian pose relative to home position

**Location:** `teleop_recorder.py`, lines 354-371

```python
# 1. Compute delta from home calibration
current_T = self.mapper.get_current_tracker_matrix()        # 4×4 matrix
T_delta = np.linalg.inv(self.mapper.tracker_home_T) @ current_T  # relative transform

# 2. Extract position delta
pos_delta = T_delta[:3, 3]  # [dx, dy, dz] in meters

# 3. Remap coordinates (Vive frame → robot frame)
remapped_pos = np.array([-pos_delta[1], -pos_delta[0], -pos_delta[2]]) * pos_scale

# 4. Extract & remap rotation
rotvec_delta = R.from_matrix(T_delta[:3, :3]).as_rotvec()  # rotation vector (axis-angle)
remapped_rotvec = np.array([-rotvec_delta[1], -rotvec_delta[0], -rotvec_delta[2]]) * rot_scale
euler_delta = R.from_rotvec(remapped_rotvec).as_euler("xyz", degrees=False)  # [rx, ry, rz]

# 5. Add to home pose
target_pose = robot_home_pose.copy()
target_pose[:3] += remapped_pos
target_pose[3:] += euler_delta
```

**Output:** `raw_pose`
- **Format:** Python list, length 6
- **Values:** `[x, y, z, rx, ry, rz]`
- **Units:** meters, radians
- **Data type:** float64
- **Logged:** not written to HDF5 in the current logger; used internally to derive `arm/safe_pose`

**Example:**
```
[-0.3220, 0.0524, 0.2382, 3.1298, 0.0616, 0.0558]
```

---

### Stage 2: Safety Bounds Filtering (Bounded Pose)

**Process:** Apply 6-step safety filter to prevent workspace violations

**Location:** `realman_utils.py`, `apply_safety_bounds()` method (copied into `robot_pose_controller.py`)

**The 6 Safety Steps:**

**Step 1: Jump Protection** (glitch filtering)
- Position: Clamp max delta to `max_pos_jump = 0.1m` per cycle
- Rotation: Slew-limit angular velocity to `max_rot_speed = 225°/s` with acceleration limit `450°/s²`

```python
# Position jump
dist = ||current - last||
if dist > max_pos_jump:
    scale = max_pos_jump / dist
    pose[:3] = last_pose[:3] + (pose[:3] - last_pose[:3]) * scale

# Rotation slew limiting (via rate command with acceleration limit)
desired_rate = (target_angle - current_angle) / dt
limited_rate = slew_limit(desired_rate, max_accel=450°/s²)
pose[3:] = current_angle + limited_rate * dt
```

**Step 2: Cartesian Box Clamp**
- Hard min/max per axis
- X: `[-0.37, 0.37]` meters
- Y: `[-0.37, 0.37]` meters  
- Z: `[0.05, 0.40]` meters (5cm off table, 40cm max height)

```python
pose[0] = max(safe_x[0], min(safe_x[1], pose[0]))  # X clamp
pose[1] = max(safe_y[0], min(safe_y[1], pose[1]))  # Y clamp
pose[2] = max(safe_z[0], min(safe_z[1], pose[2]))  # Z clamp
```

**Step 3: Reach Radius (Sphere) Clamp**
- Computed from position: `r = sqrt(x² + y² + z²)`
- Min reach: `0.15m` (prevent self-collision with base)
- Soft reach: `0.54m` (begin gradual damping)
- Max reach: `0.58m` (hard limit, RM65 max is 0.61m)

```python
radius = sqrt(x² + y² + z²)

# Soft wall: gradual pull-inward before hard limit
if soft_reach < radius <= max_reach:
    overflow = radius - soft_reach
    softened_r = radius - (soft_reach_gain * overflow)
    scale = softened_r / radius
    pose[:3] *= scale  # scale position inward

# Hard clamp
if radius > max_reach:
    scale = max_reach / radius
    pose[:3] *= scale
```

**Step 4: Minimum Reach Clamp**
```python
radius = sqrt(x² + y² + z²)
if radius < min_reach and radius > 1e-9:
    scale = min_reach / radius
    pose[:3] *= scale  # scale position outward
```

**Step 5: Orientation Wrap + Boundary Damping**
- Angles wrapped to `[-π, π]`
- Near outer boundary: damp rotation changes (prevents singularities)

```python
# Wrap each rotation to [-π, π]
for i in [3, 4, 5]:
    pose[i] = (pose[i] + π) % (2π) - π

# Boundary damping: reduce rotation speed near max radius
boundary_ratio = (radius - soft_reach) / (max_reach - soft_reach)
damp_factor = boundary_ratio * boundary_rot_damp_gain
for i in [3, 4, 5]:
    diff = shortest_angle_diff(pose[i], last_pose[i])
    pose[i] = last_pose[i] + (1 - damp_factor) * diff
```

**Output:** `filtered_pose`
- **Format:** Python list, length 6
- **Units:** meters, radians
- **Logged:** not written to HDF5 in the current logger; used internally before `arm/safe_pose`

**Example (after clamping):**
```
[-0.3220, 0.0524, 0.2305, 3.1298, 0.0616, 0.0558]  # angles wrapped
```

---

### Stage 3: Safe Pose Selection

**Process:** Choose between safety-filter output or fallback hold pose

**Location:** `teleop_recorder.py`, lines 440-445

```python
safe_pose = (
    self.mapper.apply_safety_bounds(raw_pose)
    or self.mapper.last_filtered_pose
    or self.mapper.robot_home_pose
)
```

**Output:** `safe_pose`
- **Format:** Python list, length 6
- **Values:** Safety-filtered pose, or fallback to last filtered/home pose
- **Logged:** `arm/safe_pose` in HDF5
- **Also used internally:** hold/no-hold state is not written as a separate HDF5 field in the current logger

---

### Stage 4: Arm Smoothing via EMA (Smoothed Pose)

**Process:** Exponential Moving Average interpolation for smooth motion

**Location:** `HighFrequencyInterpolator` class, lines 305-335

Uses two separate interpolations:
1. **Position (X, Y, Z):** Linear EMA with alpha=0.15
2. **Rotation (Rx, Ry, Rz):** Quaternion SLERP (shortest-path spherical interpolation)

```python
# Position: standard linear EMA
current_xyz += alpha_pos * (target_xyz - current_xyz)

# Rotation: convert to quaternions → SLERP → convert back to Euler
t = clip(alpha_rot, 0, 1)  # interpolation parameter [0, 1]
rot_current = Rotation.from_euler("xyz", current_rot, degrees=False)
rot_target = Rotation.from_euler("xyz", target_rot, degrees=False)
slerp = Slerp([0, 1], Rotation.from_quat([rot_current.as_quat(), rot_target.as_quat()]))
current_rot = slerp(t).as_euler("xyz", degrees=False)
```

**Output:** `smoothed_pose`
- **Format:** Python list, length 6
- **Units:** meters, radians
- **Logged:** `arm/smoothed_pose` in HDF5
- **This is what gets sent to the robot**

**Example:**
```
[-0.3215, 0.0531, 0.2344, 3.1210, 0.0589, 0.0587]
```

---

### Stage 1C: Hand Data Acquisition

**Source 1: MANUS Glove via ZMQ**

**Location:** `ManusErgonomicsSubscriber` class, lines 160-203

```
ZMQ PULL socket on tcp://localhost:8000 (background thread)
  → 40-value ergonomics message (comma-separated string)
  → decode & split by ","
  → convert to floats
  → extract left or right half (20 values each)
```

**Data format arriving:** String, e.g., `"1.2,3.4,5.6,..."`
**Parsed to:** Python list of 20 floats (or 40 before filtering left/right)

**Output:** `manus_joints` (raw ergonomics stream)
- **Format:** Python list, length 20
- **Values:** Finger joint angles (degrees or normalized)
- **Logged:** `hand/manus_joints` in HDF5

---

### Stage 4-H: Hand Pose Conversion (MANUS → LEAP)

**Process:** Convert MANUS ergonomics to LEAP hand joint angles

**Location:** `LeapHandDirectController.convert_manus_to_leap_pose()`, lines 262-288

**Mapping:** Direct copy approach from MANUS demo
- Extract specific indices from MANUS 20-value array
- Apply offset and gain constants
- Convert degrees → radians
- Apply allegro-to-LEAPhand transformation (π offset per joint)

```python
# Step 1: Extract & combine MANUS data with offsets
pose = deg2rad([
    manus[4:8] +                           # indices 4-7
    [manus[8] + 10] +                      # index 8 with +10° offset
    manus[9:16] +                          # indices 9-15
    [90 - 1.75 * manus[1]] +               # computed from index 1
    [-45 + 3.0 * manus[0]] +               # computed from index 0
    [-30 + 3.0 * manus[2]] +               # computed from index 2
    [manus[3]]                             # index 3
])  # Result: 16 values in radians

# Step 2: Apply per-joint gains (amplitude modulation)
pose[0] = -2.5 * pose[0] + deg2rad(20)
pose[1] = 1.5 * pose[1]
pose[4] = -2.5 * pose[4] + deg2rad(30)
pose[5] = 1.5 * pose[5]
pose[8] = -2.5 * pose[8]
pose[9] = 1.5 * pose[9]
pose[12] = 1.5 * pose[12]
pose[13] = 1.5 * pose[13] + deg2rad(90)

# Step 3: Apply allegro-to-LEAPhand transform
leap_pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
# This adds π to each joint (180° offset for motor zero position)
leap_pose = pose + π
```

**Output:** `leap_pose` (before safety clipping)
- **Format:** numpy array, length 16
- **Units:** radians
- **Range:** ~[-1.5, 3.5] radians (covers full motor span)

**Example:**
```
[3.18, 0.82, 1.43, 2.02, 3.18, 0.82, 1.43, 2.02, 3.18, 0.82, 1.43, 2.02, 1.51, 3.59, 2.35, 2.52]
```

---

### Stage 5-H: Hand Safety Clipping

**Process:** Clamp each Joint to motor limits

**Location:** `LeapHandDirectController.send_manus_command()` (calls `lhu.angle_safety_clip`)
**Defined in:** `leap_hand_utils.py`, lines 18-22

```python
def angle_safety_clip(joints):
    sim_min, sim_max = LEAPsim_limits()  # Get LEAP simulation limits
    real_min = LEAPsim_to_LEAPhand(sim_min)  # Convert to real LEAP hand frame
    real_max = LEAPsim_to_LEAPhand(sim_max)
    return np.clip(joints, real_min, real_max)  # Hard clamp each joint
```

**LEAP Hand Joint Limits (in simulation frame):**
```
MIN = [-1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, 
       -1.047, -0.314, -0.506, -0.366, -0.349, -0.47,  -1.20,  -1.34]  # radians

MAX = [1.047,  2.23,   1.885,  2.042,  1.047,  2.23,   1.885,  2.042,  
       1.047,  2.23,   1.885,  2.042,  2.094,  2.443,  1.90,   1.88]   # radians
```

(Then converted to LEAP hand frame by adding π)

**Output:** `leap_pose` (after safety clipping)
- **Format:** numpy array, length 16
- **Units:** radians
- **All values within legal motor range**
- **Logged:** `hand/leap_pose` in HDF5

---

### Stage 6: Hardware Commands

**Arm Hardware:**

**Location:** `teleop_recorder.py`, line 455

```python
arm_ret = self.mapper.robot.rm_movep_canfd(
    smoothed_pose,      # 6D Cartesian pose
    True,               # high-follow mode (≤10ms cycle)
    trajectory_mode=1,  # curve fitting mode
    radio=20            # smoothing coefficient
)
```

**Parameters:**
- `pose`: 6D list `[x, y, z, rx, ry, rz]`
- `follow`: `True` → high responsiveness (requires ≤10ms command cycle)
- `trajectory_mode`: 1 = curve fitting (smooth interpolation at arm controller)
- `radio`: 20 = moderate smoothing for live control

**Return:** Status code
- `0` = success
- `1` = controller error (bad params, bad arm state)
- `-1` = communication failure

**Hand Hardware:**

**Location:** `LeapHandDirectController.send_manus_command()`, line 294

```python
self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
```

**Process:**
1. Convert 16 joint angles (radians) to motor positions
2. Send via Dynamixel Protocol 2.0 to all 16 motors simultaneously
3. Position scale: `2.0π / 4096` radians per step (14-bit motor positions)

---

### Stage 7: Async Data Recording (HDF5 + Metadata TXT)

**File Creation:**

**Location:** `HDF5LoggingProcess.run()` in `teleop_recorder.py`

```python
file = h5py.File(output_path, "w")

datasets = {
    "time/monotonic_s":         (N,)            float64         perf_counter() at loop start
    "arm/raw_pose":             (N, 6)          float64         unfiltered tracker delta (for learning/analysis)
    "arm/safe_pose":            (N, 6)          float64         pose after safety bounds / hold fallback
    "arm/smoothed_pose":        (N, 6)          float64         after EMA smoothing -> sent to robot
    "hand/manus_joints":        (N, 20)         float64         raw glove ergonomics, or NaNs if absent
    "hand/leap_pose":           (N, 16)         float64         converted LEAP command, or NaNs if absent
    "camera/color":             (N, H, W, 3)    uint8           BGR frames (ML-optimized: only unique frames)
    "camera/timestamp_ns":      (N,)            uint64          timestamps of unique camera frames
    "camera/frame_indices":     (N,)            int32           mapping from sample i to frame index (NEW)
}

# All datasets use chunks=True for streaming writes.
# All datasets use maxshape=(None, ...) for unbounded growth.
# camera/color uses LZF compression to reduce file size.
# NEW: camera/frame_indices eliminates duplicate frames → ML-ready
```

**Sample Append:**

**Location:** logger process main loop (`log_queue.get(...)`)

```python
# ML-OPTIMIZED: Only unique frames are stored in camera/color
# Each telemetry sample maps to a frame via camera/frame_indices

frame_count = 0  # Tracks unique camera frames
current_frame_idx = -1  # Index in camera/color dataset

# When a new frame arrives:
if cam_ts != last_logged_cam_ts and cam_ts != 0:
    ds_c.resize((frame_count + 1, ...))  # Expand camera/color array
    ds_c[frame_count] = new_frame        # Write only once
    current_frame_idx = frame_count
    frame_count += 1

# Every telemetry sample gets a mapping:
idx = sample_count
ds_fi.resize((idx + 1,))
ds_fi[idx] = current_frame_idx  # Points to which camera frame applies
sample_count += 1
```

**Example (ML-ready structure):**
```
Telemetry samples:     T1  T2  T3  T4  T5  T6  T7  T8  ...  (100+ Hz @ N samples)
Camera frames:         F0  ═══════════  F1  ═══════════  (30 Hz @ M unique frames, M << N)

Stored in HDF5:
/camera/color:         [F0, F1, ...] (M frames, compressed)
/camera/frame_indices: [0,  0,  0,  0,  1,  1,  1,  1, ...]  (N indices)
/camera/timestamp_ns:  [ts0, ts1, ...]  (M timestamps)

For ML training:
- NO DATA CORRUPTION: Each frame paired with exactly one state
- COMPACT: F0 stored once, not 4 times
- TRAINABLE: Skip duplicate states via frame_indices
```

**File Attributes (Metadata):**

```python
file.attrs["created_utc"]          = "2026-04-19T21:22:17.010276Z"
file.attrs["robot_ip"]             = "192.168.1.18"
file.attrs["control_hz"]           = 125.0
file.attrs["sample_count"]         = 1584  # written on close()
file.attrs["total_time_seconds"]   = 12.66  # written on close()
```

**Stage 7B: Metadata TXT Sidecar**

In addition to HDF5 attributes, the logger writes a human-readable metadata summary file next to the recording.

Default naming (auto mode):
- HDF5: `logs/teleop_data_N.hdf5`
- TXT: `logs/teleop_metadata_N.txt`

Custom path mode (`--log-path my_run.hdf5`):
- HDF5: `my_run.hdf5`
- TXT: `my_run.txt`

**Metadata TXT File Structure:**

```
teleop_metadata_0.txt
├── Recording Date (UTC)
├── Recording Date (Local)
├── Control Parameters
│   ├── Control Frequency (Hz)
│   ├── Interpolation Decay (Position)
│   └── Interpolation Decay (Rotation)
├── Recording Statistics
│   ├── Total Runtime (seconds)
│   └── Total Samples
└── Recording Config
    └── Camera Enabled
```

**Contents:**
- recording date in UTC and local time
- control and interpolation parameters
- runtime statistics such as total samples
- whether camera was enabled for this recording
- no robot, hand, or file-name fields are written to the TXT sidecar

**Example:**
```
================== Teleop Recording Metadata ==================
Recording Date (UTC): 2026-04-28T14:25:25.123456Z
Recording Date (Local): 2026-04-28T10:25:25.123456

--- Control Parameters ---
Control Frequency (Hz): 125.0
Interpolation Decay (Position): 0.15
Interpolation Decay (Rotation): 0.15

--- Recording Statistics ---
Total Runtime (seconds): 45.23
Total Samples: 5654

--- Recording Config ---
Camera Enabled: False
============================================================
```

**Final HDF5 File Structure:**

```
teleop_data_0.hdf5
├── time/
│   └── monotonic_s          (N,)          perf timestamps (one per sample)
├── arm/
│   ├── raw_pose             (N, 6)        unfiltered tracker (for learning)
│   ├── safe_pose            (N, 6)        final choice (move or hold)
│   └── smoothed_pose        (N, 6)        after EMA -> sent to hardware
├── hand/
│   ├── manus_joints         (N, 20)       raw glove data or NaNs
│   └── leap_pose            (N, 16)       converted LEAP command or NaNs
├── camera/
│   ├── color                (M, H, W, 3)  unique BGR frames (M << N, ML-optimized!)
│   ├── timestamp_ns         (M,)          frame timestamps (M unique frames)
│   └── frame_indices        (N,)          mapping: sample i → frame index

KEY INSIGHT: frame_indices[i] tells you which camera frame applies to sample i.
This eliminates 3-4x data duplication while maintaining frame sync!
```

---

## Pipeline 2: Replay from HDF5

### Stage R1: File Loading

**Location:** `replay_hdf5.py`, `_load_recording()`, lines 64-73

```python
with h5py.File(hdf5_path, "r") as f:
    arm_poses   = f["arm/smoothed_pose"][:]     # Load entire array into memory (N,)
    hand_poses  = f["hand/leap_pose"][:]        # (N,)
    timestamps  = f["time/monotonic_s"][:]      # For timing (N,)
    meta        = dict(f.attrs)                 # Metadata dict
    has_glove   = ~np.isnan(hand_poses[:, 0])
    has_camera  = "camera/color" in f
    
    # ML-optimized format support:
    if "camera/frame_indices" in f:
        colors = f["camera/color"][:]           # Load unique frames (M,)
        frame_indices = f["camera/frame_indices"][:]  # Mapping (N,)
    else:
        colors = f["camera/color"][:]           # Legacy: N frames
        frame_indices = np.arange(len(arm_poses))  # Default 1:1 mapping
```

**Data in memory:**
- `arm_poses`: numpy array, shape `(N, 6)`, dtype float64 (loaded from `arm/smoothed_pose`)
- `hand_poses`: numpy array, shape `(N, 16)`, dtype float64
- `colors`: numpy array, shape `(M, H, W, 3)` or `(N, H, W, 3)`, dtype uint8 (**ML-optimized: M unique frames**)
- `frame_indices`: numpy array, shape `(N,)`, dtype int32 (**maps each sample to camera frame**)
- `timestamps`: numpy array, shape `(N,)`, dtype float64
- `meta`: dict with HDF5 file attributes such as `created_utc`, `robot_ip`, `control_hz`, `sample_count`, `frame_count` (if camera enabled), and `total_time_seconds`
- `has_camera`: bool indicating whether `camera/color` is present in the file

**Note:** The HDF5 file also contains `arm/raw_pose` (unfiltered tracker data) for potential analysis or policy learning. Replay always uses `arm/smoothed_pose` to reproduce the original smooth trajectory.

The metadata TXT sidecar is informational for operators and experiment logs; replay reads the HDF5 file only.

---

### Stage R2: Robot Connection & Initialization

**Location:** `replay_hdf5.py`, lines 152-159

```python
ctrl = RobotPoseController(
    robot_ip=robot_ip,
    robot_port=robot_port,
    hand_port=hand_port,
    connect_hand=connect_hand,
)

# Inside RobotPoseController.__init__():
#   1. Connect to RealMan arm via TCP
#   2. Read current arm pose (via forward kinematics)
#   3. Connect to LEAP hand via serial
#   4. Initialize safety filter (stateful, tracks velocity/acceleration)
```

**Safety Filter Initialization:**

```python
self.safety = ArmSafetyFilter()
current_pose = ctrl._get_current_arm_pose()
# current_pose is queried from robot via rm_get_joint_degree() + FK
```

---

### Stage R3: Safety Filter Re-Seeding

**Location:** `replay_hdf5.py`, lines 167-175

```python
first_valid_pose = None
for i in range(n):
    if not np.any(np.isnan(arm_poses[i])):
        first_valid_pose = arm_poses[i].tolist()
        break

if first_valid_pose:
    ctrl.safety.seed(first_valid_pose)
    # Sets last_filtered_pose = first_valid_pose
    # This prevents huge jump clamps when replaying from a different start position
```

**Why this matters:**
- During recording, poses came from live tracker (could be anywhere in workspace)
- During replay, we might start with robot in a different position
- Seeding with first replay pose prevents `max_rot_jump = 0.85 rad` from clamping every motion

---

### Stage R4: Homing Phase (Smooth Move to Start Position)

**Location:** `replay_hdf5.py`, lines 177-213

```python
# Get current position (where robot is now)
current_pose = ctrl._get_current_arm_pose()

# Create 50 interpolated poses from current → first_replay_pose
homing_poses = _interpolate_linear_pose(current_pose, first_valid_pose, steps=50)

# Send each homing pose over ~2 seconds
for step, pose in enumerate(homing_poses):
    ret = ctrl.send_arm_pose(
        pose.tolist(),
        trajectory_mode=2,  # filter mode for smooth motion
        trajectory_radio=500  # high smoothing
    )
    # Space out commands: step/50 * 2 seconds
```

**Interpolation Method:**

```python
def _interpolate_linear_pose(start, end, steps):
    poses = []
    rot_start = Rotation.from_euler("xyz", start[3:], degrees=False)
    rot_end = Rotation.from_euler("xyz", end[3:], degrees=False)
    slerp = Slerp([0, 1], Rotation.from_quat([rot_start.as_quat(), rot_end.as_quat()]))
    
    for i in range(steps + 1):
        alpha = i / steps
        pos = start[:3] + alpha * (end[:3] - start[:3])  # linear
        rot = slerp(alpha).as_euler("xyz", degrees=False)  # SLERP
        poses.append(np.concatenate([pos, rot]))
    return poses
```

**Result:** Robot smoothly moves to the starting pose of the recording over ~2 seconds, preventing motor strain.

---

### Stage R5: Replay Loop with Timing

**Location:** `replay_hdf5.py`, lines 218-248

```python
replay_wall_start = time.perf_counter()

for i in range(n):
    loop_t = time.perf_counter()
    
    arm_pose = arm_poses[i]  # shape (6,)
    hand_pose = hand_poses[i] if has_glove[i] else None  # shape (16,) or None
    
    # Skip frames with NaN (can occur at logger startup)
    if np.any(np.isnan(arm_pose)):
        continue
    
    # Send arm with trajectory smoothing (mode 2, radio 500 by default)
    ret = ctrl.send_arm_pose(
        arm_pose.tolist(),
        trajectory_mode=2,
        trajectory_radio=500
    )
    
    # Send hand only if glove data was active during recording
    if hand_pose is not None and not np.any(np.isnan(hand_pose)):
        ctrl.send_hand_pose(hand_pose.tolist())
    
    # Maintain original timing (scaled by speed factor)
    if i + 1 < n:
        original_dt = timestamps[i + 1] - timestamps[i]  # e.g., 0.008 sec @ 125 Hz
        target_dt = original_dt / speed  # e.g., 0.016 sec if speed=0.5
        target_t = loop_t + target_dt
        
        # Precision spin-wait (busy-wait)
        while time.perf_counter() < target_t:
            pass
```

**Timing Example:**
- Original recording at 125 Hz: `dt = 1/125 = 0.008s` between samples
- Replay at 1.0x speed: send every 0.008s
- Replay at 0.5x speed (half speed): send every 0.016s
- Replay at 2.0x speed (double speed): send every 0.004s

---

### Stage R6: Safety Filtering During Replay

**In each `send_arm_pose()` call:**

```python
def send_arm_pose(self, pose_6d, trajectory_mode=1, trajectory_radio=20):
    pose_list = list(np.asarray(pose_6d, dtype=np.float64))
    safe = self.safety.apply(pose_list)  # Apply safety filter again
    return self.robot.rm_movep_canfd(safe, True, trajectory_mode, trajectory_radio)
```

**Why filter again?**
- Data was filtered during recording (went through `apply_safety_bounds`)
- But replay might be interrupted or have communication issues
- Safety filter provides **second layer of protection**
- Will mostly pass through smoothly since data is already safe

**What gets filtered:**
1. **Jump clamps**: Any sudden pose jumps (shouldn't happen during normal replay)
2. **Boundary damping**: As arm approaches workspace limits
3. **Rotation slew limiting**: Gradual acceleration/deceleration of rotations

---

### Stage R7: Hardware Commands (Same as Live)

**Arm:**
```python
ret = self.robot.rm_movep_canfd(safe, True, trajectory_mode=2, trajectory_radio=500)
```
- Mode 2 (filter) instead of mode 1 (curve-fitting) for smoother prerecorded motion
- `radio=500` for high smoothing (vs. `radio=20` for live responsiveness)

**Hand:**
```python
self.hand.write_desired_pos(motors, clipped_joints)
```
- Same as live control
- Sends 16 joint angles to Dynamixel motors

---

## Verification & Sync Testing

### verify_sync.py: Proving Frame Synchronization

The `verify_sync.py` script demonstrates that frames are correctly synced with robot state, even with ML-optimized compact storage.

**How it Proves Sync (ML-Optimized Format):**

```python
# Load mapping (NEW: frame_indices)
frame_indices = f['camera/frame_indices'][:]  # For each sample i, which frame?
colors = f['camera/color'][:]                 # Unique frames only

# During playback:
for i in range(num_samples):
    frame_idx = frame_indices[i]  # Gets the ACTUAL frame for sample i
    frame = colors[frame_idx]     # Correct frame, not duplicated
```

**On-Screen Verification:**
- **Frame ID** displayed: Shows which unique frame (0, 1, 2, ...) is being viewed
- **Sample ID** displayed: Shows which telemetry sample (0, 1, 2, ...)
- Compare Frame ID across samples: You'll see "Frame ID: 0" for multiple samples, then jump to "Frame ID: 1"
- This visually proves: Multiple robot states → same visual frame (no corruption!)
- **Trajectory plot** with playhead cursor: Shows when frame changes occur relative to arm motion

**Example Playback Output:**
```
Time: 0.00s  Sample: 0/1200   Frame ID: 0  ← Frame 0 appears for ~4 samples
Time: 0.05s  Sample: 1/1200   Frame ID: 0  
Time: 0.10s  Sample: 2/1200   Frame ID: 0
Time: 0.15s  Sample: 3/1200   Frame ID: 0
Time: 0.20s  Sample: 4/1200   Frame ID: 1  ← Frame 1 replaces it (~33ms later)
Time: 0.25s  Sample: 5/1200   Frame ID: 1
...
```

**Why This Proves Sync (vs. Duplicates):**
1. **No ML corruption**: 1 frame → 1 state (same timestamp)
2. **Timestamp consistency**: frame_indices[i] always points to correct frame
3. **Compact proof**: Visual shows sparse Frame ID changes (not every sample new)
4. **Deterministic**: Same Frame ID for all samples between updates

---

## Data Flow Diagram

```
LIVE TELEOPERATION PIPELINE
━━━━━━━━━━━━━━━━━━━━━━━━━━

Vive Tracker                    MANUS Glove (ZMQ)                RealSense Camera
        ↓                                  ↓                                 ↓
    4×4 Transform             40-value ergonomics stream            RGB frame + timestamp
        ↓                                  ↓                                 ↓
    Extract position          Split left/right (20 values)         [camera/color] +
    Remap coordinates               ↓                               [camera/timestamp_ns]
        ↓                        [manus_joints] (logged)
    [raw_pose] (computed)           ↓
        ↓                        Convert to LEAP frame
  
  ├── Safety Bounds (6 steps)    ├── Apply safety clip
  │   • Jump protection          │
  │   • Slew limiting            └── [leap_pose] (logged)
  │   • Box clamp                    ↓
  │   • Reach clamp             [send to 16 Dynamixels]
  │   • Min reach                    ↓
  │   • Boundary damping        LEAP Hand Hardware
  │
  ├→ Choose move or hold
  │
  ├→ [safe_pose] (logged)
  │
  └→ EMA + SLERP smoothing
      ↓
     [smoothed_pose] (logged)
      ↓
     [send to rm_movep_canfd]
      ↓
     RealMan Arm Hardware


HDF5 LOGGING
━━━━━━━━━━━━
All steps logged to file in parallel:
- arm/{raw_pose, safe_pose, smoothed_pose}
- hand/{manus_joints, leap_pose}
- camera/{timestamp_ns, color} when camera logging is enabled
- time/{monotonic_s}
- plus sidecar metadata text file: teleop_metadata_N.txt


REPLAY PIPELINE
━━━━━━━━━━━━━━

HDF5 File
    ↓
Load all data into memory
    ↓
Connect to arm + hand
    ↓
Seed safety filter with first replay pose
    ↓
HOMING PHASE (50 interpolated steps over 2 sec)
    Current position → First recorded position
    ├→ Linear interpolation for X, Y, Z
    └→ SLERP interpolation for Rx, Ry, Rz
    ↓
[arm at starting pose]
    ↓
REPLAY LOOP (for each sample i in 1..N)
    ├→ arm_poses[i] → safety filter → rm_movep_canfd
    ├→ hand_poses[i] → safety clip → write_desired_pos
    ├→ Wait for next sample time (using timestamps)
    └→ Repeat @ original Hz (scaled by --speed)
    ↓
[trajectory complete]
```

---

## Key Data Types & Shapes Reference

| Data | Shape | Type | Range | Units | Notes |
|------|-------|------|-------|-------|-------|
| `raw_pose` | (N, 6) or (6,) | float64 | any | m, rad | unfiltered tracker |
| `safe_pose` | (N, 6) or (6,) | float64 | safe only | m, rad | after bounds check |
| `smoothed_pose` | (N, 6) or (6,) | float64 | safe/smooth | m, rad | sent to hardware |
| `manus_joints` | (N, 20) or (20,) | float64 | ~[0, 1] | normalized | raw glove data |
| `leap_pose` | (N, 16) or (16,) | float64 | [-3, 4] | rad | LEAP hand command |
| `camera/timestamp_ns` | (M,) | uint64 | any | ns | **M unique frames** |
| `camera/color` | (M, H, W, 3) | uint8 | 0-255 | BGR image | **ML-optimized: M < N** |
| `camera/frame_indices` | (N,) | int32 | 0..M-1 | index | **NEW: sample i → frame index** |
| `time/monotonic_s` | (N,) | float64 | any | seconds | one per sample |

---

## Summary: What Happens at Each Processing Stage

1. **Raw acquisition** → Tracker/glove streaming live
2. **Coordinate transform** → Vive frame → Robot frame
3. **Safety filtering** → 6-step bounds check (prevents collisions)
4. **Pose selection** → Move or hold (if bounds violated)
5. **Smoothing** → EMA + SLERP (eliminates jitter)
6. **Hardware command** → Cartesian arm + 16D hand
7. **Logging** → All intermediate stages captured to HDF5
8. **Replay** → Load HDF5 → Homing → Play at original speed

