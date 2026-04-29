# Bimanual Teleoperation with Recording & Replay

Real-time control of a RealMan RM65 arm + LEAP Hand via Vive tracker + MANUS glove, with asynchronous HDF5 recording plus a human-readable metadata TXT sidecar.

## Quick Start

### Record a trajectory
```bash
python src/teleop_recorder.py
# Move Vive tracker to control arm, operate MANUS glove for hand
# Ctrl+C to stop
# Files saved: src/logs/teleop_data_N.hdf5 and src/logs/teleop_metadata_N.txt (recording date/time, control frequency, interpolation decay, runtime stats)
```

### Replay a trajectory
```bash
python src/replay_hdf5.py src/logs/teleop_data_0.hdf5
# Arm homes to start position, then replays motion
```

## Documentation

- **[INSTALLATION_AND_RUN.md](src/documentation/INSTALLATION_AND_RUN.md)** — Canonical lab setup guide for the MANUS SDK and combined teleop logger
- **[src/documentation/EXECUTION_OPTIONS.md](src/documentation/EXECUTION_OPTIONS.md)** — Execution-only command and parameter reference
- **[src/documentation/DATA_PIPELINE.md](src/documentation/DATA_PIPELINE.md)** — Detailed explanation of data flow, processing stages, and file formats

## Core Scripts

| Script | Purpose |
|--------|---------|
| `teleop_recorder.py` | Live teleoperation with HDF5 logging (main script) |
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
✅ **Multi-stage logging** — raw, safe, smoothed arm poses  
✅ **Hand safety** — automatically clips LEAP joint angles  
✅ **Smooth replay** — homing phase + trajectory smoothing  
✅ **Data inspection** — HDF5 datasets plus metadata TXT summary  
✅ **Hardware agnostic** — works with any poses via `robot_pose_controller.py`  

## Hardware

- **Arm:** RealMan RM65 (Ethernet, 192.168.1.18:8080)
- **Hand:** LEAP Hand with 16 Dynamixel XH motors (serial, USB)
- **Tracking:** Vive Tracker (SteamVR, 6DOF position)
- **Glove:** MANUS with ergonomics on ZMQ (tcp://localhost:8000)

## What Gets Recorded

Each HDF5 file contains:
- **Arm:** raw_pose, safe_pose, smoothed_pose
- **Hand:** manus_joints (raw), leap_pose (converted)
- **Time:** monotonic_s
- **Camera:** timestamp_ns and optional color frames when `--enable-camera` is used
- **HDF5 attributes:** created_utc, robot_ip, control_hz, sample_count, total_time_seconds

Each recording also writes a compact metadata text sidecar:
- **TXT sidecar:** `teleop_metadata_N.txt` (or `<log-path>.txt` when `--log-path` is used)
- **TXT fields:** recording date in UTC/local time, control frequency, interpolation decay values, total runtime, total samples, and whether camera was enabled

See `src/documentation/DATA_PIPELINE.md` for full schema details.

## Workflow Example

```bash
# 1. Record
python src/teleop_recorder.py
# (3-second countdown, then teleoperate for ~30s)
# Ctrl+C
# → src/logs/teleop_data_0.hdf5 and src/logs/teleop_metadata_0.txt

# 2. Inspect
python src/replay_hdf5.py src/logs/teleop_data_0.hdf5 --dry-run
# → Samples: 3750, Duration: 30s, First pose: [...], Samples with glove: 3750/3750

# 3. Test replay (half speed)
python src/replay_hdf5.py src/logs/teleop_data_0.hdf5 --speed 0.5
# → 2s homing + 60s replay at half speed

# 4. Full speed replay
python src/replay_hdf5.py src/logs/teleop_data_0.hdf5
# → 2s homing + 30s replay at real-time
```

## Troubleshooting

**"Failed to connect to RealMan arm"**  
→ Check arm powered on and at 192.168.1.18:8080. See [INSTALLATION_AND_RUN.md](src/documentation/INSTALLATION_AND_RUN.md).

**"No Vive Trackers found"**  
→ Start SteamVR and pair tracker in controller settings.

**"Waiting for MANUS ergonomics data"**  
→ Start MANUS app and enable ergonomics broadcast on tcp://localhost:8000.

See run-order and issue notes in [INSTALLATION_AND_RUN.md](src/documentation/INSTALLATION_AND_RUN.md).

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

Use `src/documentation/DATA_PIPELINE.md` for:
- Exact data flow with code locations
- Processing stage details
- Data type & shape reference
- HDF5 schema documentation
- Visual diagrams of the pipeline

Use `src/documentation/EXECUTION_OPTIONS.md` for:
- All CLI flags and options
- Execution examples for each runnable script

Use `src/documentation/INSTALLATION_AND_RUN.md` for:
- Hardware and software setup
- Dependency installation
- Run order and common issue handling

---

**Questions?** Refer to the detailed guides above or check the code comments in the source files.
