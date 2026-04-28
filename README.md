# Bimanual Teleoperation with Recording & Replay

Real-time control of a RealMan RM65 arm + LEAP Hand via Vive tracker + MANUS glove, with complete HDF5 recording of all trajectories and pipeline stages.

## Quick Start

### Record a trajectory
```bash
python combined_simple_teleop_real_logger.py
# Move Vive tracker to control arm, operate MANUS glove for hand
# Ctrl+C to stop
# File saved: logs/teleop_YYYYMMDD_HHMMSS.hdf5
```

### Replay a trajectory
```bash
python replay_hdf5.py logs/teleop_YYYYMMDD_HHMMSS.hdf5
# Arm homes to start position, then replays motion
```

## Documentation

- **[INSTALLATION_AND_RUN.md](INSTALLATION_AND_RUN.md)** — Canonical lab setup guide for the MANUS SDK and combined teleop logger
- **[SETUP_AND_EXECUTION.md](SETUP_AND_EXECUTION.md)** — Complete setup guide, installation, and all command options
- **[DATA_PIPELINE.md](DATA_PIPELINE.md)** — Detailed explanation of data flow, processing stages, and file formats

## Core Scripts

| Script | Purpose |
|--------|---------|
| `combined_simple_teleop_real_logger.py` | Live teleoperation with HDF5 logging (main script) |
| `replay_hdf5.py` | Replay trajectories from HDF5 files with smooth homing |
| `robot_pose_controller.py` | Standalone controller for sending poses + safety bounds |

## System Architecture

```
Live Teleoperation          Replay from HDF5
─────────────────          ────────────────

Vive Tracker                HDF5 File
    ↓                          ↓
6D Position              Load poses + timing
    ↓                          ↓
Safety Filter ───→ HDF5 ←─ Safety Filter
    ↓                          ↓
Smoothing                  Arm + Hand commands
    ↓                          ↓
Hardware commands         Physical motion
```

## Key Features

✅ **Real-time control** at 125 Hz with safety bounds  
✅ **Multi-stage logging** — raw, bounded, safe, smoothed poses  
✅ **Hand safety** — automatically clips LEAP joint angles  
✅ **Smooth replay** — homing phase + trajectory smoothing  
✅ **Data inspection** — HDF5 with full metadata and timing  
✅ **Hardware agnostic** — works with any poses via `robot_pose_controller.py`  

## Hardware

- **Arm:** RealMan RM65 (Ethernet, 192.168.1.18:8080)
- **Hand:** LEAP Hand with 16 Dynamixel XH motors (serial, USB)
- **Tracking:** Vive Tracker (SteamVR, 6DOF position)
- **Glove:** MANUS with ergonomics on ZMQ (tcp://localhost:8000)

## What Gets Recorded

Each HDF5 file contains:
- **Arm:** raw_pose, bounded_pose, safe_pose, smoothed_pose, hold_flag, canfd_status
- **Hand:** manus_joints (raw), leap_pose (converted), has_glove_data
- **Time:** monotonic_s, wall_time_s
- **Metadata:** robot IP/port, control Hz, home pose, tracker calibration, etc.

See `DATA_PIPELINE.md` for full schema details.

## Workflow Example

```bash
# 1. Record
python combined_simple_teleop_real_logger.py
# (3-second countdown, then teleoperate for ~30s)
# Ctrl+C
# → logs/teleop_20260419_162217.hdf5

# 2. Inspect
python replay_hdf5.py logs/teleop_20260419_162217.hdf5 --dry-run
# → Samples: 3750, Duration: 30s, First pose: [...], Samples with glove: 3750/3750

# 3. Test replay (half speed)
python replay_hdf5.py logs/teleop_20260419_162217.hdf5 --speed 0.5
# → 2s homing + 60s replay at half speed

# 4. Full speed replay
python replay_hdf5.py logs/teleop_20260419_162217.hdf5
# → 2s homing + 30s replay at real-time
```

## Troubleshooting

**"Failed to connect to RealMan arm"**  
→ Check arm powered on and at 192.168.1.18:8080. See [SETUP_AND_EXECUTION.md](SETUP_AND_EXECUTION.md#troubleshooting).

**"No Vive Trackers found"**  
→ Start SteamVR and pair tracker in controller settings.

**"Waiting for MANUS ergonomics data"**  
→ Start MANUS app and enable ergonomics broadcast on tcp://localhost:8000.

See full troubleshooting guide in [SETUP_AND_EXECUTION.md](SETUP_AND_EXECUTION.md#troubleshooting).

## Key Concepts

**Safety Filter (6 stages):**
1. Jump protection (glitch filtering)
2. Rotation slew limiting (smooth accelerations)
3. Cartesian box clamp (X, Y, Z bounds)
4. Reach radius clamp (soft + hard limits)
5. Minimum reach clamp (prevent self-collision)
6. Boundary rotation damping (near singularities)

**Smoothing:**
- Position: Exponential Moving Average (alpha=0.15)
- Rotation: Quaternion SLERP (shortest-path interpolation)

**Hand Control:**
- MANUS 20-value ergonomics → LEAP 16-joint angles
- Safety-clipped to motor limits before sending to Dynamixels

**Replay:**
- Homing phase: smooth interpolation from current → first recorded pose (2s)
- Replay: send poses at original timestamps, maintaining cadence
- Speed control: multiply all inter-frame delays by `--speed` factor

## For Your Research

Use `DATA_PIPELINE.md` for:
- Exact data flow with code locations
- Processing stage details
- Data type & shape reference
- HDF5 schema documentation
- Visual diagrams of the pipeline

Use `SETUP_AND_EXECUTION.md` for:
- Complete hardware & software setup
- All CLI flags and options
- File format examples
- Troubleshooting procedures
- Best practices & safety guidelines

---

**Questions?** Refer to the detailed guides above or check the code comments in the source files.
