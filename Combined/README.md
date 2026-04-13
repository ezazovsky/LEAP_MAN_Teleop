# Combined Teleop

This folder contains a new standalone teleoperation entry point that leaves
`RealMan-main` and `RealManus-LEAPHand-main` unchanged.

## Script

- `combined_simple_teleop.py`
- `combined_realsense_teleop.py`

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
