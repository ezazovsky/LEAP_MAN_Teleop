# System Data-Flow Blueprint

This document is a concise blueprint for a UML-style system diagram of the Combined teleoperation + recording + replay system. It lists the runtime components, their inputs/outputs, the APIs used at each handoff, communication channels, data formats, and final outputs.

Use this as the authoritative mapping when drawing your diagram.

---

## Top-level Components

- Teleop Controller (process): `Combined/combined_simple_teleop_real_logger.py` (main)
- Vive Tracker Adapter: `track.ViveTrackerModule` (used by `teleoperate.ViveToRMMapper`)
- RM Robot API: `Robotic_Arm.rm_robot_interface` (RMAPI Python wrapper)
- MANUS Glove Adapter: `ManusErgonomicsSubscriber` (ZMQ PULL client)
- LEAP Hand Controller: `LeapHandDirectController` (uses `leap_hand_utils` + `DynamixelClient`)
- Camera Process: `RealSenseCamera` wrapped by `CameraProcess` (pyrealsense2)
- HDF5 Logger Process: `HDF5LoggingProcess` (h5py writer)
- Replay Tool: `replay_hdf5.py` (reader + `RobotPoseController`)

---

## Data Inputs (External Sources)

- Vive Tracker (SteamVR): raw 4×4 transform matrices → obtained via `ViveTrackerModule.get_T()`
- MANUS glove ergonomics stream: comma-separated string broadcast over ZMQ (tcp://localhost:8000)
- RealSense RGB camera: color frames + per-frame timestamp via `pyrealsense2`
- Robot telemetry: current joint angles and FK via `Robotic_Arm` API calls (e.g., `rm_get_joint_degree`, `rm_algo_forward_kinematics`)

---

## Intra-System Channels & Transport

- ZMQ (tcp://localhost:8000): MANUS ergonomics broadcast → consumed by `ManusErgonomicsSubscriber` (PULL socket)
- Local function calls / library APIs:
  - `ViveToRMMapper` calls `ViveTrackerModule.get_T()` and RMAPI functions such as `rm_create_robot_arm`, `rm_get_joint_degree`, `rm_movep_canfd`.
  - `LeapHandDirectController` calls functions in `leap_hand_utils` and methods on `DynamixelClient`.
- Multiprocessing Queue (`mp.Queue`): teleop main loop → `HDF5LoggingProcess` for low-latency logging. Items are dicts with `camera_ts` and `arrays` (numpy arrays).
- Shared Memory (`multiprocessing.shared_memory`): CameraProcess writes RGB buffer to a shared memory block named `cam_color_shm`; `HDF5LoggingProcess` reads from this block to capture frames without copying via IPC.

---

## Component-by-Component Flow (detailed)

1) Vive Tracker → ViveToRMMapper (teleop)
   - Input: 4×4 transform matrix from `ViveTrackerModule.get_T()`
   - API / code: `teleoperate.ViveToRMMapper.get_current_tracker_matrix()`
   - Processing: compute delta vs `tracker_home_T`, remap axes, convert rotation → rotation vector → Euler (`scipy.spatial.transform.Rotation`)
   - Output: `raw_pose` (6-element list: [x,y,z,rx,ry,rz], meters & radians)

2) Safety Filtering (ViveToRMMapper.apply_safety_bounds)
   - Input: `raw_pose` (from step 1)
   - API / code: `ViveToRMMapper.apply_safety_bounds()`
   - Processing: 6-step safety (jump protection, box clamp, reach radius soft/hard clamp, min reach, orientation wrap/damping)
   - Output: `safe_pose` (6-element list) — used for logging and smoothing

3) Smoothing / Interpolator (teleop main)
   - Input: `safe_pose`
   - API / code: `HighFrequencyInterpolator.step()` (EMA for position; quaternion SLERP for rotation)
   - Output: `smoothed_pose` (6-element list) — sent to RM robot

4) Robot Control (RMAPI)
   - Input: `smoothed_pose`
   - API: `Robotic_Arm` Python wrapper via calls such as `rm_movep_canfd(pose, follow, mode, radio)`
   - Transport: TCP to RealMan controller at configured `robot_ip:robot_port`
   - Effect: arm moves; controller returns status codes

5) MANUS Glove → ManusErgonomicsSubscriber (ZMQ)
   - Input: ergonomics CSV string (40 values) over ZMQ
   - API / code: `ManusErgonomicsSubscriber` (background thread) reads, parses, and stores last message
   - Output: `manus_joints` (20-value list) available to main loop

6) MANUS → LEAP Hand conversion
   - Input: `manus_joints` (20 floats)
   - API / code: `LeapHandDirectController.send_manus_command()` calls mapping functions in `leap_hand_utils` (e.g., `allegro_to_LEAPhand`) and applies offsets/gains
   - Output: `leap_pose` (16-element joint angles, radians)

7) LEAP Hand → Dynamixel motors
   - Input: `leap_pose` (post-clipping)
   - API / code: `DynamixelClient` methods (connect, `write_desired_pos`, `set_torque_enabled`)
   - Transport: Serial (USB) connection to Dynamixel bus
   - Effect: physical hand motors update

8) Camera capture path (optional)
   - Producer: `RealSenseCamera` (pyrealsense2)
   - Component: `CameraProcess` writes frames into shared memory (`cam_color_shm`) and stores `timestamp_ns` in a shared value
   - Consumer: `HDF5LoggingProcess` reads shared memory when `camera_ts` indicates a new frame and writes `camera/color` and `camera/timestamp_ns` datasets

9) Teleop Logging (async)
   - Producer: teleop main loop constructs `log_data` dict each cycle containing `camera_ts` and `arrays` (numpy arrays for time, arm/raw_pose, arm/safe_pose, arm/smoothed_pose, hand/manus_joints, hand/leap_pose)
   - Transport: `mp.Queue` (`log_queue.put_nowait(log_data)`)
   - Consumer: `HDF5LoggingProcess` (`log_queue.get(timeout=0.5)`) resizes and appends to datasets using `h5py`.
   - File format: HDF5 with dataset names:
     - `time/monotonic_s` (float64)
     - `arm/raw_pose`, `arm/safe_pose`, `arm/smoothed_pose` (float64 arrays (N,6))
     - `hand/manus_joints` (float64 (N,20))
     - `hand/leap_pose` (float64 (N,16))
     - `camera/timestamp_ns` (uint64 (N,)), `camera/color` (uint8 (N,H,W,3) compressed LZF)
   - Sidecar: `HDF5LoggingProcess._write_metadata_file(...)` writes a compact human-readable TXT sidecar paired with the HDF5 (`teleop_metadata_N.txt` or `<log-path>.txt`)

10) Replay (offline consumer)
    - Input: HDF5 file (`teleop_data_N.hdf5`)
    - API / code: `replay_hdf5.py` loads datasets via `h5py.File(..., 'r')` and uses `RobotPoseController` (an RMAPI wrapper) to send poses back to the robot via `rm_movep_canfd` and to send hand poses via the same hand controller codepath.
    - Logic: seed safety filter with first valid pose, perform 2s homing interpolation, replay poses at original timestamps (scaled by `--speed`), apply safety checks again before each send.

---

## Message/Data Shape Summary (compact)

- Vive Tracker: 4×4 float64 matrix → `raw_pose`: list[float] (6)
- Manus Glove: CSV string (40) → `manus_joints`: list[float] (20)
- Leap Pose: list[float] (16) radians → motor positions via DynamixelClient
- Teleop Log item (mp.Queue entry):
  - `camera_ts`: int (ns)
  - `arrays`: dict[str -> numpy.ndarray]
    - keys: `time/monotonic_s`, `arm/raw_pose`, `arm/safe_pose`, `arm/smoothed_pose`, `hand/manus_joints`, `hand/leap_pose`

---

## Key APIs & Libraries to Show in Diagram

- SteamVR / Vive Tracker: `track.ViveTrackerModule` (project adapter)
- RMAPI Robot: `Robotic_Arm` (rm_create_robot_arm, rm_get_joint_degree, rm_movep_canfd, rm_delete_robot_arm)
- MANUS ergonomics: ZMQ PULL (tcp://localhost:8000) → `ManusErgonomicsSubscriber`
- LEAP hand utils: `leap_hand_utils` (conversions and safety clip functions)
- Dynamixel: `leap_hand_utils.dynamixel_client.DynamixelClient` (serial motor control)
- Camera: `pyrealsense2` → `RealSenseCamera`
- IPC: `multiprocessing.Queue` (teleop→logger), `multiprocessing.shared_memory` (`cam_color_shm`)
- Storage: `h5py` (HDF5 datasets + attributes) and plain text file for metadata sidecar

---

## Suggested Diagram Elements

- Components (boxes): Teleop Main, Vive Tracker Adapter, RMAPI Robot, MANUS ZMQ Adapter, LeapHand Controller, Camera Process, HDF5 Logger, Replay Tool
- Channels (arrows labeled with transport & shape): ZMQ, shared_memory, mp.Queue, TCP (RMAPI), Serial (Dynamixel), function calls (internal adapters)
- Data artifacts (files/DBs): `teleop_data_N.hdf5`, `teleop_metadata_N.txt`
- Notes: mark which flows are synchronous (robot commands) vs asynchronous (logging, camera write)

---

## Minimal textual swimlane (quick reference)

Teleop Main → (call) ViveTracker.get_T() → compute `raw_pose` → (call) `apply_safety_bounds()` → `safe_pose` → (call) interpolator → `smoothed_pose` → (RPC/TCP) `rm_movep_canfd(smoothed_pose)` → Robot

MANUS App (external) → (ZMQ publish) ergonomics CSV → (ZMQ) `ManusErgonomicsSubscriber` → `LeapHandDirectController.send_manus_command()` → (serial) DynamixelClient.write_desired_pos() → Hand motors

Camera (RealSense) → CameraProcess → shared_memory `cam_color_shm` → HDF5LoggingProcess → `camera/color` dataset in HDF5

Teleop main → mp.Queue.put(log_data) → HDF5LoggingProcess.consume → append datasets + write TXT sidecar

Replay Tool reads HDF5 → RobotPoseController → seed safety → home to start → replay via `rm_movep_canfd` and Leap hand writes

---

If you want, I can:
- generate a simple PlantUML skeleton file from this blueprint, or
- add a diagram-ready JSON (nodes/edges) export you can load into a visual tool.

---

File: Combined/SYSTEM_BLUEPRINT.md


## ASCII Diagram (quick reference)

Below is an easy-to-read ASCII diagram you can use directly or copy into your notes as a starting layout for a UML/system diagram.

```
LIVE TELEOPERATION PIPELINE
───────────────────────────

Vive Tracker                MANUS Glove (ZMQ)                      RealSense Camera
4×4 Transform               40-value ergonomics stream             RGB frame + timestamp
    │                          │                                      │
    ▼                          ▼                                      ▼
 Extract position        Split left/right (20 values)         [camera/color] + [camera/timestamp_ns]
 Remap coords                 │                                      │
    │                         ▼                                      │
    ▼                   [manus_joints] (20)                           │
 [raw_pose] (6)               │                                      │
    │                         ▼                                      │
    ├─► Safety Bounds (6 steps) ────────────────────┐                  │
    │   - Jump protection                            │                  │
    │   - Box clamp                                  │                  │
    │   - Reach clamp                                │                  │
    │   - Min reach                                  │                  │
    │   - Orientation damping                        │                  │
    │                                                │                  │
    ▼                                                ▼                  ▼
 [safe_pose] (6)  ──► EMA + SLERP smoothing ──► [smoothed_pose] (6)      │
    │                                                │                  │
    │                                                │                  │
    │                                                ▼                  │
    │                                           rm_movep_canfd()         │
    │                                                │                  │
    │                                                ▼                  │
    │                                         RealMan Arm Hardware        │
    │                                                                    │
    │                                                                    │
    └────────────────────────────────────────────────────────────────────┘

Additional Flows:
- MANUS stream -> Leap conversion -> DynamixelClient -> LEAP Hand motors
- CameraProcess -> shared_memory(cam_color_shm) -> HDF5LoggingProcess -> camera/color dataset
- Teleop main -> mp.Queue (log_data) -> HDF5LoggingProcess -> teleop_data_N.hdf5
- HDF5LoggingProcess writes paired teleop_metadata_N.txt sidecar

Replay Flow (offline):
teleop_data_N.hdf5 -> replay_hdf5.py -> RobotPoseController -> seed safety -> homing -> replay via rm_movep_canfd

```
