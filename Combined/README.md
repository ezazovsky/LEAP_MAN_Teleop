# Combined Teleop

This folder contains a new standalone teleoperation entry point that leaves
`RealMan-main` and `RealManus-LEAPHand-main` unchanged.

## Scripts

- `combined_simple_teleop.py` — live teleoperation (Vive + MANUS glove)
- `combined_simple_teleop_real_logger.py` — same as above but writes an HDF5 log
- `combined_realsense_teleop.py` — live teleoperation with RealSense camera logging
- `robot_pose_controller.py` — standalone controller: send any pose to the hardware with full safety bounds
- `replay_hdf5.py` — replay a recorded HDF5 file on the real robot

This version uses:

- Vive tracker for RealMan arm motion
- MANUS 40-value ergonomics data for LEAP Hand finger motion
- plain Python only
- direct joint copying for the hand side

It does **not** use ROS2 or PyBullet IK.

The RealSense variant adds:

- Intel RealSense RGB capture
- one RGB frame logged for each teleop sample
- camera timestamps and intrinsics stored in the HDF5 log

## What It Reuses

- `RealMan-main/teleoperate.py` for arm tracking, calibration, filtering, and safety
- `RealManus-LEAPHand-main/Bidex_Manus_Teleop/python/leap_hand_utils` for LEAP motor control helpers
- MANUS ZMQ ergonomics stream on `tcp://localhost:8000`

## Run

From the repo root:

```powershell
python .\Combined\combined_simple_teleop.py
```

Example with explicit options:

```powershell
python .\Combined\combined_simple_teleop.py --robot-ip 192.168.1.18 --robot-port 8080 --zmq-endpoint tcp://localhost:8000 --hand-side right
```

Enable HDF5 logging during a demo:

```powershell
python .\Combined\combined_simple_teleop.py --log-hdf5
```

Run the RealSense-enabled variant:

```powershell
python .\Combined\combined_realsense_teleop.py --log-hdf5
```

Example with explicit camera settings:

```powershell
python .\Combined\combined_realsense_teleop.py --log-hdf5 --camera-width 640 --camera-height 480 --camera-fps 30
```

Choose a specific output file:

```powershell
python .\Combined\combined_simple_teleop.py --log-hdf5 --log-path .\Combined\logs\demo_01.hdf5
```

---

## robot_pose_controller.py

A reusable controller that accepts a 6D Cartesian arm pose and 16-joint LEAP hand
pose, runs them through the full safety-bounds pipeline (same as live teleop), and
sends the commands to the hardware.  It can also be imported as a Python module by
`replay_hdf5.py` or any other script.

### Quick connection test (no movement)

Connects to the arm and hand, reads the current pose, then disconnects:

```bash
python Combined/robot_pose_controller.py --no-hand
```

### Send a single arm pose

The pose is `[x y z rx ry rz]` in **meters** and **radians**.
Safety bounds are always applied before the command reaches the hardware.

```bash
python Combined/robot_pose_controller.py --pose 0.3 0.0 0.2 0.0 0.0 0.0
```

With custom robot IP and no hand:

```bash
python Combined/robot_pose_controller.py \
    --robot-ip 192.168.1.18 \
    --robot-port 8080 \
    --pose 0.3 0.0 0.2 0.0 0.0 0.0 \
    --no-hand
```

### CLI flags

| Flag | Default | Description |
|------|---------|-------------|
| `--robot-ip` | `192.168.1.18` | RealMan arm IP |
| `--robot-port` | `8080` | RealMan arm TCP port |
| `--hand-port` | auto | Serial port for LEAP hand (`/dev/ttyUSB0` etc.) |
| `--pose X Y Z Rx Ry Rz` | — | Single 6D pose to send (meters, radians) |
| `--no-hand` | off | Skip LEAP hand connection |

---

## replay_hdf5.py

Reads an HDF5 file produced by `combined_simple_teleop_real_logger.py` and physically
replays the recorded arm and hand movements on the real robot at the original speed.

**What it feeds to the robot:**
- `arm/smoothed_pose` (shape `N × 6`) → arm, through `robot_pose_controller` safety bounds
- `hand/leap_pose` (shape `N × 16`) → LEAP hand, only on frames where `has_glove_data` was `True`

**Timing:** uses `time/monotonic_s` from the file to reproduce the original inter-frame
cadence exactly.  The `--speed` flag scales all delays uniformly.

### Inspect a file without touching hardware

```bash
python Combined/replay_hdf5.py Combined/logs/teleop_20240101_120000.hdf5 --dry-run
```

Output shows sample count, duration, first/last arm pose, and glove-data coverage.

### Real-time replay (arm + hand)

```bash
python Combined/replay_hdf5.py Combined/logs/teleop_20240101_120000.hdf5
```

The robot will hold still for 3 seconds (countdown printed), then follow the recorded trajectory.

### Replay arm only (skip hand)

```bash
python Combined/replay_hdf5.py Combined/logs/teleop_20240101_120000.hdf5 --no-hand
```

### Half-speed replay

```bash
python Combined/replay_hdf5.py Combined/logs/teleop_20240101_120000.hdf5 --speed 0.5
```

### Custom robot IP + longer countdown

```bash
python Combined/replay_hdf5.py Combined/logs/teleop_20240101_120000.hdf5 \
    --robot-ip 192.168.1.18 \
    --robot-port 8080 \
    --start-delay 5
```

### CLI flags

| Flag | Default | Description |
|------|---------|-------------|
| `hdf5_path` | required | Path to the `.hdf5` recording |
| `--robot-ip` | `192.168.1.18` | RealMan arm IP |
| `--robot-port` | `8080` | RealMan arm TCP port |
| `--hand-port` | auto | Serial port for LEAP hand |
| `--speed` | `1.0` | Playback speed multiplier (0.5 = half, 2.0 = double) |
| `--no-hand` | off | Skip LEAP hand replay |
| `--dry-run` | off | Print file summary, do NOT connect to hardware |
| `--start-delay` | `3.0` | Seconds to count down before motion starts |

---

## Assumptions

- SteamVR and the Vive tracker are already running
- MANUS SDK is already publishing 40-value ergonomics data over ZMQ
- The RealMan Python SDK is available to Python as `Robotic_Arm.rm_robot_interface`
- The LEAP Hand is connected on one of `/dev/ttyUSB0`, `/dev/ttyUSB1`, or `COM13`, unless `--hand-port` is provided

## Notes

- The hand mapping is intentionally the simple direct-copy version from the MANUS demo path.
- `--hand-side left` will read the left 20 ergonomics values from ZMQ, but the direct-copy mapping was originally tuned for the right-hand demo, so left-hand behavior may need follow-up adjustment.
- If `--log-hdf5` is enabled, the script writes a streaming HDF5 log file and flushes it every `--log-flush-every` samples.
- `combined_realsense_teleop.py` waits for a fresh RealSense color frame each loop, so the effective sample rate is camera-limited if `--control-hz` is higher than `--camera-fps`.

## HDF5 Contents

The logger stores extendable datasets so samples are appended while the demo is running.
For HDF Viewer compatibility, it now writes both grouped datasets and a flat root-level
table dataset.

- `/samples`

- `/time/monotonic_s`
- `/time/wall_time_s`
- `/arm/raw_pose`
- `/arm/bounded_pose`
- `/arm/safe_pose`
- `/arm/smoothed_pose`
- `/arm/hold_flag`
- `/arm/canfd_status`
- `/hand/manus_joints`
- `/hand/leap_pose`
- `/hand/has_glove_data`
- `/camera/frame_number`
- `/camera/timestamp_ms`
- `/camera/capture_time_s`
- `/camera/has_frame`
- `/camera/rgb`

If your viewer does not render the grouped datasets clearly, open `/samples` first.
It contains one row per control-loop sample with all arm and hand fields together.

For `combined_realsense_teleop.py`, `/samples` also includes camera timing metadata,
while the RGB arrays are stored under `/camera/rgb` with shape
`[num_samples, height, width, 3]`.

It also stores metadata as file attributes, including:

- `robot_ip`
- `robot_port`
- `zmq_endpoint`
- `hand_side`
- `control_hz`
- `robot_home_pose`
- `tracker_home_T`
- `sample_count`
