# Bimanual Teleoperation: Execution Guide

This file is execution-only. It lists which scripts you can run from `src/` and which parameters each script accepts.

## Run Location

Run commands from this directory:

```bash
cd /home/fri/FRIRobot-1/src
```

## Runnable Scripts

## 1) `teleop_recorder.py`

Purpose: live teleoperation (arm + hand) with optional camera capture and HDF5 logging.

Basic run:

```bash
python teleop_recorder.py
```

Default recording outputs (paired):
- `logs/teleop_data_N.hdf5`
- `logs/teleop_metadata_N.txt` with recording date/time, control frequency, interpolation decay, and runtime statistics

If `--log-path` is provided (for example `--log-path ./logs/run_01.hdf5`), the sidecar metadata file is written as `./logs/run_01.txt` with the same compact fields.

Example with custom tuning:

```bash
python teleop_recorder.py \
  --robot-ip 192.168.1.18 \
  --robot-port 8080 \
  --zmq-endpoint tcp://localhost:8000 \
  --hand-side right \
  --control-hz 125 \
  --arm-pos-scale 1.0 \
  --arm-rot-scale 1.0 \
  --calibration-countdown 3 \
  --log-path ./logs/session_001.hdf5 \
  --log-flush-every 50 \
  --enable-camera
```

Parameters:

| Flag | Type | Default | Notes |
|---|---|---|---|
| `--robot-ip` | string | `192.168.1.18` | Robot controller IP |
| `--robot-port` | int | `8080` | Robot controller port |
| `--zmq-endpoint` | string | `tcp://localhost:8000` | MANUS ergonomics ZMQ endpoint |
| `--hand-side` | choice | `right` | `left` or `right` |
| `--hand-port` | string | `None` | Serial port; auto-detect if omitted |
| `--control-hz` | float | `125.0` | Main loop frequency |
| `--calibration-countdown` | int | `3` | Seconds before teleop starts |
| `--arm-pos-scale` | float | `1.0` | Position sensitivity multiplier |
| `--arm-rot-scale` | float | `1.0` | Rotation sensitivity multiplier |
| `--hand-current-limit` | int | `350` | LEAP hand current limit (mA) |
| `--log-hdf5` | bool flag | `True` | Present in parser; effectively on by default |
| `--log-path` | string | `None` | Output file path; auto-numbered if omitted |
| `--log-flush-every` | int | `50` | Flush interval in samples |
| `--enable-camera` | bool flag | `False` | Enable RealSense color stream logging |

## 2) `replay_hdf5.py`

Purpose: replay recorded trajectories from an HDF5 log.

Basic run:

```bash
python replay_hdf5.py logs/teleop_data_0.hdf5
```

Useful examples:

```bash
# Inspect only, no hardware commands
python replay_hdf5.py logs/teleop_data_0.hdf5 --dry-run

# Half-speed replay
python replay_hdf5.py logs/teleop_data_0.hdf5 --speed 0.5

# Replay without hand
python replay_hdf5.py logs/teleop_data_0.hdf5 --no-hand

# Replay with recorded camera video window
python replay_hdf5.py logs/teleop_data_0.hdf5 --show-video
```

Parameters:

| Arg/Flag | Type | Default | Notes |
|---|---|---|---|
| `hdf5_path` | positional string | required | Input recording file |
| `--robot-ip` | string | `192.168.1.18` | Robot controller IP |
| `--robot-port` | int | `8080` | Robot controller port |
| `--hand-port` | string | `None` | Serial port override |
| `--speed` | float | `1.0` | Replay speed multiplier |
| `--no-hand` | bool flag | `False` | Skip hand replay |
| `--dry-run` | bool flag | `False` | Load/inspect file only |
| `--start-delay` | float | `3.0` | Countdown before replay |
| `--trajectory-mode` | int | `2` | Controller trajectory mode |
| `--trajectory-radio` | int | `500` | Smoothing coefficient |
| `--show-video` | bool flag | `False` | Show recorded camera feed during replay |

## 3) `robot_pose_controller.py`

Purpose: send a single 6D arm pose (and optionally connect hand).

Basic run:

```bash
python robot_pose_controller.py --pose 0.25 0.10 0.20 0.0 0.0 0.0
```

Arm-only example:

```bash
python robot_pose_controller.py --no-hand --pose 0.30 0.00 0.20 0.0 0.0 0.0
```

Parameters:

| Flag | Type | Default | Notes |
|---|---|---|---|
| `--robot-ip` | string | `192.168.1.18` | Robot controller IP |
| `--robot-port` | int | `8080` | Robot controller port |
| `--hand-port` | string | `None` | Serial port override |
| `--pose` | 6 floats | none | `X Y Z Rx Ry Rz` (m, rad) |
| `--no-hand` | bool flag | `False` | Do not connect LEAP hand |

## 4) `verify_sync.py`

Purpose: visualize logged camera frames and arm trajectory for sync inspection.

Basic run:

```bash
python verify_sync.py logs/teleop_data_0.hdf5
```

Faster playback:

```bash
python verify_sync.py logs/teleop_data_0.hdf5 --speed 2.0
```

Parameters:

| Arg/Flag | Type | Default | Notes |
|---|---|---|---|
| `hdf5_file` | positional string | required | Input recording file |
| `--speed` | float | `1.0` | Playback speed multiplier |

## Non-CLI Modules In This Folder

These files are used as imported modules and are not primary command-line entry scripts in their current form:

- `realman_utils.py`
- `track.py`

## Quick Command Reference

```bash
# Live teleop + logging
python teleop_recorder.py

# Replay recording
python replay_hdf5.py logs/teleop_data_0.hdf5

# Single pose command
python robot_pose_controller.py --pose 0.25 0.10 0.20 0.0 0.0 0.0

# Sync visualizer
python verify_sync.py logs/teleop_data_0.hdf5
```
