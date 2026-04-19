# Bimanual Teleoperation: Setup & Execution Guide

Complete guide to setting up and running the teleoperation system for recording and replaying arm + hand control trajectories.

---

## System Overview

This system enables real-time teleoperation of a RealMan RM65 robotic arm with a LEAP Hand using:
- **Vive Tracker** (SteamVR) for 6D arm control
- **MANUS Glove** (ZMQ ergonomics stream) for 16-joint hand control
- **HDF5 logging** to record both modalities with all safety pipeline stages
- **Replay capability** to reproduce recorded trajectories with motor protection

**Key files:**
- `combined_simple_teleop_real_logger.py` — Live teleoperation with HDF5 logging
- `robot_pose_controller.py` — Standalone controller for any 6D pose + hand command
- `replay_hdf5.py` — Replay recorded trajectories from HDF5 files

---

## Physical Setup Prerequisites

### Hardware Required

1. **RealMan RM65 Arm**
   - IP: 192.168.1.18 (default, configurable)
   - Network: Connected to control PC via Ethernet
   - Power: 24V supply, emergency stop accessible

2. **LEAP Hand (16 joints)**
   - Serial connection: `/dev/ttyUSB0`, `/dev/ttyUSB1`, or `COM13`
   - Current limit: 350mA (default, adjustable)

3. **Vive Tracker (SteamVR)**
   - Vive Base Stations calibrated and powered on
   - Tracker bound to SteamVR in controller settings
   - At least 1.5m × 1.5m play space

4. **MANUS Glove with Ergonomics Streaming**
   - Broadcasting 40-value ergonomics on ZMQ: `tcp://localhost:8000`
   - Dual-side data: left 20 values (indices 0-19) + right 20 values (indices 20-39)

5. **Control PC**
   - Linux or Windows
   - Python 3.7+
   - Network access to 192.168.1.18 (arm)
   - USB ports for hand serial (if not using USB-to-network)

---

## Software Installation

### Step 1: Clone & Dependencies

```bash
cd /path/to/repo
pip install h5py numpy scipy
```

### Step 2: RealMan Python SDK

The RealMan SDK must be available to Python as an importable package:

```bash
# Option A: Already installed system-wide (check if this works)
python3 -c "from Robotic_Arm.rm_robot_interface import *; print('OK')"

# Option B: SDK in non-standard location
# Add to ~/.bashrc or ~/.zshrc:
export PYTHONPATH="/path/to/RealMan/SDK:$PYTHONPATH"
```

**Verify arm connection:**
```bash
python3 -c "
import sys
sys.path.insert(0, '/path/to/RealMan-main')
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle = robot.rm_create_robot_arm('192.168.1.18', 8080)
print(f'Connected: {handle.id != -1}')
robot.rm_delete_robot_arm()
"
```

### Step 3: LEAP Hand & Dynamixel

Already included in repo as `RealManus-LEAPHand-main/`:

```bash
# Test LEAP hand connection
python3 -c "
import sys
sys.path.insert(0, 'RealManus-LEAPHand-main/Bidex_Manus_Teleop/python')
from leap_hand_utils.dynamixel_client import DynamixelClient
dxl = DynamixelClient(list(range(16)), '/dev/ttyUSB0', 4_000_000)
dxl.connect()
print('LEAP hand connected on /dev/ttyUSB0')
dxl.disconnect()
"
```

### Step 4: OpenVR & Vive Tracker

```bash
pip install openvr
# SteamVR must be running on the system or network-accessible
```

**Test Vive connection:**
```bash
python3 -c "
import sys
sys.path.insert(0, 'RealMan-main')
from track import ViveTrackerModule
vive = ViveTrackerModule()
devices = vive.return_selected_devices('tracker')
print(f'Trackers found: {list(devices.keys())}')
"
```

### Step 5: MANUS Ergonomics (ZMQ)

```bash
pip install zmq
```

**Verify MANUS streaming:**
```bash
# In another terminal, run MANUS application and start ergonomics broadcast
# Then test receiving:
python3 -c "
import zmq
ctx = zmq.Context()
sock = ctx.socket(zmq.PULL)
sock.connect('tcp://localhost:8000')
sock.setsockopt(zmq.RCVTIMEO, 1000)  # 1s timeout
msg = sock.recv().decode('utf-8')
values = msg.split(',')
print(f'Received {len(values)} values: {values[:5]}...')
"
```

---

## Pre-Execution Checklist

Before running any teleoperation code:

- [ ] RealMan arm powered on at 192.168.1.18
- [ ] LEAP hand powered on and connected to `/dev/ttyUSB[0-1]`
- [ ] Vive Base Stations powered and tracking
- [ ] Vive Tracker bound and visible in SteamVR
- [ ] MANUS glove powered and streaming ergonomics on `tcp://localhost:8000`
- [ ] SteamVR process running on this PC or accessible over network
- [ ] Network connectivity: this PC can ping 192.168.1.18
- [ ] Serial port accessible: `ls -l /dev/ttyUSB*` shows LEAP hand
- [ ] Play area clear of obstacles (at least 1.5m × 1.5m)
- [ ] Emergency stop button accessible

---

## Running Live Teleoperation with Logging

### Default usage (HDF5 logging enabled by default)

```bash
cd /home/fri/FRIRobot-main/Combined
python combined_simple_teleop_real_logger.py
```

**What happens:**
1. Connects to arm at 192.168.1.18:8080
2. Initializes Vive tracker and waits for SteamVR
3. Connects to LEAP hand on first available port
4. Subscribes to MANUS ergonomics on ZMQ
5. **3-second countdown** — hold Vive tracker steady
6. **Teleoperation active** — move tracker to control arm, operate glove to control hand
7. **Press Ctrl+C** to stop
8. HDF5 file saved to `Combined/logs/teleop_YYYYMMDD_HHMMSS.hdf5`

### With custom robot IP

```bash
python combined_simple_teleop_real_logger.py --robot-ip 192.168.1.18 --robot-port 8080
```

### Without LEAP hand (arm only)

```bash
python combined_simple_teleop_real_logger.py --hand-port /dev/null
# or skip hand connection with modified code
```

### Specify output file location

```bash
python combined_simple_teleop_real_logger.py --log-path ./logs/my_demo_001.hdf5
```

### Full option list

| Flag | Default | Description |
|------|---------|-------------|
| `--robot-ip` | `192.168.1.18` | RealMan arm IP |
| `--robot-port` | `8080` | RealMan arm TCP port |
| `--zmq-endpoint` | `tcp://localhost:8000` | MANUS ZMQ address |
| `--hand-side` | `right` | Which glove side: `left` or `right` |
| `--hand-port` | auto-detect | LEAP hand serial port (tries USB0/USB1/COM13) |
| `--hand-current-limit` | `350` | Dynamixel current limit (mA) |
| `--control-hz` | `125.0` | Control loop frequency (Hz) |
| `--arm-pos-scale` | `1.0` | Position sensitivity multiplier |
| `--arm-rot-scale` | `1.0` | Rotation sensitivity multiplier |
| `--calibration-countdown` | `3` | Seconds before control starts |
| `--log-hdf5` | `True` | Enable HDF5 logging (default) |
| `--log-path` | auto | Output HDF5 file path |
| `--log-flush-every` | `50` | Flush HDF5 to disk every N samples |

---

## Understanding the Live Teleoperation Output

While running, you'll see:

```
Combined teleop running at 125.0 Hz. Press Ctrl+C to stop.
HDF5 logging enabled: Combined/logs/teleop_20260419_162217.hdf5

Calibrating in 3 seconds. Hold tracker steady!
3...
2...
1...

Home pose XYZ: -0.322, 0.052, 0.238
Home pose RPY: 3.130, 0.062, 0.056
Calibration Complete!
Robot Anchor: [-0.3220, 0.0524, 0.2382, 3.1298, 0.0616, 0.0558]

Waiting for MANUS ergonomics data on ZMQ...
Arm: -0.3215 0.0523 0.2355 3.1254 0.0589 0.0587   hand:on
```

**Interpreting the output:**
- **Home pose**: Arm's Cartesian position when you hold tracker steady
- **RPY**: Roll-Pitch-Yaw in radians (multiply by 57.3 for degrees)
- **Arm**: Real-time smoothed pose being sent to robot
- **hand:on**: Glove data actively streaming; `hand:wait` when not available

**Boundary clamping messages:**
```
[BOUNDS CLAMPED:CARTESIAN] in=(...) out=(...)
```
This means you moved the tracker outside the safe workspace. The arm stopped at the boundary.

---

## Recording HDF5 Data

HDF5 logging is automatically enabled. The file contains:

**Arm trajectory data:**
- `arm/raw_pose` — Unfiltered tracker input
- `arm/bounded_pose` — After safety bounds applied
- `arm/safe_pose` — Final pose chosen
- `arm/smoothed_pose` — After EMA smoothing (what's sent to robot)
- `arm/hold_flag` — Whether motion was clamped
- `arm/canfd_status` — Command success/failure code

**Hand trajectory data:**
- `hand/manus_joints` — Raw glove ergonomics (20 values)
- `hand/leap_pose` — Converted LEAP hand angles (16 values, radians)
- `hand/has_glove_data` — Was glove active this frame?

**Timing:**
- `time/monotonic_s` — Loop timing (for replay)
- `time/wall_time_s` — UNIX timestamp

**Metadata (file attributes):**
- `robot_ip`, `robot_port` — Hardware addresses
- `control_hz` — Control frequency
- `robot_home_pose` — Calibration position
- `tracker_home_T` — Calibration transformation
- `sample_count` — Total samples recorded

**File location:** `Combined/logs/teleop_YYYYMMDD_HHMMSS.hdf5`

---

## Replaying Recorded Trajectories

### Basic replay (real-time, smooth motion)

```bash
python replay_hdf5.py Combined/logs/teleop_20260419_162217.hdf5
```

**What happens:**
1. Connects to arm and hand
2. **Homing phase** (~2 seconds) — smoothly moves arm from current position to the first recorded pose
3. **3-second countdown**
4. **Replay** — executes the full recorded trajectory at original speed
5. **Complete** — arm returns to starting pose or stops

### Half-speed replay (easier to watch)

```bash
python replay_hdf5.py logs/teleop_YYYYMMDD_HHMMSS.hdf5 --speed 0.5
```

### Arm-only replay (no hand)

```bash
python replay_hdf5.py logs/teleop_YYYYMMDD_HHMMSS.hdf5 --no-hand
```

### Maximum smoothing (best for pre-recorded data)

```bash
python replay_hdf5.py logs/teleop_YYYYMMDD_HHMMSS.hdf5 \
    --trajectory-mode 2 \
    --trajectory-radio 800
```

### Inspect file without connecting to hardware

```bash
python replay_hdf5.py logs/teleop_YYYYMMDD_HHMMSS.hdf5 --dry-run
```

**Output:**
```
[DRY RUN] No hardware connection made.

  Samples            : 1584
  Recording duration : 12.664 s
  First arm pose     : [-0.2884, 0.0306, 0.2437, -3.1223, 0.0371, 0.1004]
  Last  arm pose     : [-0.3220, 0.0524, 0.2382, 3.1150, 0.0521, 0.0440]
  Samples with glove : 1584 / 1584
```

### Full replay option list

| Flag | Default | Description |
|------|---------|-------------|
| `hdf5_path` | required | Path to the `.hdf5` file |
| `--robot-ip` | `192.168.1.18` | RealMan arm IP |
| `--robot-port` | `8080` | RealMan arm TCP port |
| `--hand-port` | auto-detect | LEAP hand serial port |
| `--speed` | `1.0` | Playback speed (0.5 = half, 2.0 = double) |
| `--no-hand` | off | Skip LEAP hand replay |
| `--dry-run` | off | Inspect file, don't connect hardware |
| `--start-delay` | `3.0` | Countdown before replay (seconds) |
| `--trajectory-mode` | `2` | Arm command mode (0=passthrough, 1=curve-fit, 2=filter) |
| `--trajectory-radio` | `500` | Smoothing coefficient |

---

## Sending a Single Pose Directly

Test arm movement without teleoperation:

```bash
python robot_pose_controller.py --no-hand --pose 0.3 0.0 0.2 0.0 0.0 0.0
```

**Format:** `--pose X Y Z Rx Ry Rz` (meters, radians)

**Example — move to a specific position:**
```bash
python robot_pose_controller.py --pose 0.25 0.1 0.15 0.0 0.0 0.0
```

---

## Troubleshooting

### "Failed to connect to RealMan arm"
```
ERROR: Failed to connect to RealMan arm.
```
- Check arm is powered on
- Verify IP address: `ping 192.168.1.18`
- Check Ethernet connection
- Try providing explicit IP: `--robot-ip <IP>`

### "No Vive Trackers found"
```
ERROR: RuntimeError: No Vive Trackers found!
```
- Ensure SteamVR is running
- Pair tracker in SteamVR controller settings
- Restart SteamVR if tracker not recognized

### "Failed to connect to LEAP Hand"
```
ERROR: Failed to connect to LEAP hand: [details]
```
- Check USB connection: `ls -l /dev/ttyUSB*`
- Try manual port: `--hand-port /dev/ttyUSB0`
- Verify baud rate is 4Mbps
- Restart hand firmware if necessary

### "Waiting for MANUS ergonomics data" (endless wait)
```
Waiting for MANUS ergonomics data on ZMQ...
```
- Ensure MANUS app is running and broadcasting
- Check ZMQ endpoint: `--zmq-endpoint tcp://localhost:8000`
- Verify no firewall blocking localhost:8000
- Test ZMQ manually: `python3 -c "import zmq; ctx=zmq.Context(); sock=ctx.socket(zmq.PULL); sock.connect('tcp://localhost:8000'); print(sock.recv())"`

### "CANFD transmission error"
```
CANFD transmission error: -1
```
- Communication timeout — check arm responsiveness
- Reduce `--control-hz` to lower bandwidth demand
- Check network stability

### "Arm stutters or doesn't move smoothly during replay"
- Use `--trajectory-mode 2 --trajectory-radio 800` for maximum smoothing
- Reduce `--speed 0.5` to move slower (easier to track)
- Check arm is not hitting workspace boundaries (look for `[BOUNDS CLAMPED]` messages)

### "arm/smoothed_pose is NaN in HDF5 file"
- Arm disconnected mid-recording
- NaN values are safe — replay script skips them
- Check `arm/canfd_status` for error codes

---

## Inspecting HDF5 Files

### Using Python (in repo directory)

```python
import h5py
import numpy as np

with h5py.File('logs/teleop_YYYYMMDD_HHMMSS.hdf5', 'r') as f:
    # Print metadata
    print("Recording info:")
    for key in f.attrs:
        print(f"  {key}: {f.attrs[key]}")
    
    # Plot arm trajectory
    import matplotlib.pyplot as plt
    poses = f['arm/smoothed_pose'][:]
    
    plt.figure(figsize=(12, 4))
    plt.subplot(1, 2, 1)
    plt.plot(poses[:, 0], label='X')
    plt.plot(poses[:, 1], label='Y')
    plt.plot(poses[:, 2], label='Z')
    plt.xlabel('Sample')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.title('Arm Position Trajectory')
    
    plt.subplot(1, 2, 2)
    plt.plot(poses[:, 3], label='Rx')
    plt.plot(poses[:, 4], label='Ry')
    plt.plot(poses[:, 5], label='Rz')
    plt.xlabel('Sample')
    plt.ylabel('Rotation (rad)')
    plt.legend()
    plt.title('Arm Rotation Trajectory')
    
    plt.tight_layout()
    plt.savefig('trajectory.png')
    plt.show()
```

### Using HDFView (GUI)
```bash
# Install: sudo apt install hdfview
hdfview logs/teleop_YYYYMMDD_HHMMSS.hdf5 &
```

---

## Tips & Best Practices

**Recording Tips:**
- Ensure smooth, deliberate movements (avoid jerky motions)
- Keep tracker within ~1.5m × 1.5m working volume
- Don't exceed workspace boundaries (you'll see `[BOUNDS CLAMPED]` messages)
- Move at moderate speed (~0.3 m/s) for best results
- Keep glove visible and ergonomics streaming

**Replay Tips:**
- Always use `--dry-run` first to inspect file before replay
- Use `--start-delay 5` if you need time to get clear of the arm
- Test with `--speed 0.5` first, then increase to full speed
- For maximum smoothness, use `--trajectory-mode 2 --trajectory-radio 800`
- Disable hand with `--no-hand` if testing arm only

**Safety:**
- Always have emergency stop accessible
- Keep play area clear before starting any script
- Don't stand directly in front of arm during replay
- Monitor first few seconds of any motion
- Start with slow speeds (`--speed 0.5`) for untested recordings

---

## Directory Structure

```
Combined/
├── combined_simple_teleop_real_logger.py   Live teleop + HDF5 logging
├── robot_pose_controller.py                Standalone pose controller
├── replay_hdf5.py                          Replay script
├── README.md                               This file
├── DATA_PIPELINE.md                        Detailed data pipeline documentation
├── logs/                                   HDF5 recordings (auto-created)
│   ├── teleop_20260419_162217.hdf5
│   ├── teleop_20260419_165412.hdf5
│   └── ...
└── __pycache__/                           (ignored)
```

---

## Quick Start Flowchart

```
START
  ↓
[Check all hardware is powered & connected]
  ↓
python combined_simple_teleop_real_logger.py
  ↓
[Wait for "Calibration Complete!"]
  ↓
[Hold Vive tracker steady and operate glove]
  ↓
[Press Ctrl+C when done]
  ↓
HDF5 file saved to logs/
  ↓
python replay_hdf5.py logs/teleop_YYYYMMDD_HHMMSS.hdf5 --dry-run
  ↓
[Review file—sample count, duration, poses]
  ↓
python replay_hdf5.py logs/teleop_YYYYMMDD_HHMMSS.hdf5 --speed 0.5
  ↓
[Arm homes to start pose, then replays]
  ↓
END
```

