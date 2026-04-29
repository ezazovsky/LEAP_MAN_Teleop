# FRIRobot Installation and Run Guide

This document captures the minimum setup required to run the two pieces of this project that depend on external software:

1. The MANUS Core 2.4.0 SDK client under `LMAPI/`
2. The combined teleoperation logger under `src/`

The steps below are written for Ubuntu-based Linux systems, which is the recommended environment for this repository.

---

## 1. Prerequisites

Before installing anything, make sure the following hardware and access are available:

- A MANUS glove and a valid MANUS SDK license
- A RealMan RM65 arm reachable on the network
- A LEAP Hand connected over USB serial
- A Vive Tracker and SteamVR for tracker input
- Internet access for package installation

Recommended software baseline:

- Ubuntu 22.04 or newer
- Python 3.9 or newer
- Git

---

## 2. Install the MANUS SDK Client

The repository already includes the MANUS Core 2.4.0 SDK client under `LMAPI/MANUS_Core_2.4.0_SDK/SDKClient_Linux/`, so you do not need to download a separate copy for the Linux build.

### 2.1 System packages

Install the native build and ZMQ dependencies first:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  git \
  cmake \
  libtool \
  libzmq3-dev \
  libusb-1.0-0-dev \
  libudev-dev \
  libncurses5-dev \
  gdb
```

If you are using SteamVR on the same machine, also install the tracker/runtime packages used by the teleop stack:

If you need to run SteamVR without a headset attached, follow [STEAMGUIDE.md](STEAMGUIDE.md) for the null-driver setup.

```bash
sudo apt-get install -y \
  steam \
  libsdl2-dev \
  libvulkan-dev \
  libssl-dev \
  zlib1g-dev
```

### 2.2 Build the SDK client

The Linux client links against `lzmq`, which is already reflected in the included `Makefile`.

```bash
cd /home/rmtest/FRIRobot/LMAPI/MANUS_Core_2.4.0_SDK/SDKClient_Linux
make
```

If the build succeeds, the executable will be created as `SDKClient_Linux.out` in the same directory.

### 2.3 Run the MANUS client

Start the client from the same directory:

```bash
./SDKClient_Linux.out
```

For a single-machine Linux setup, use the integrated standalone mode and keep the default ZMQ endpoint at `tcp://127.0.0.1:8000`.

If you are using a Windows machine as the MANUS Core host and a separate Linux machine as the client, keep the MANUS Core instance on Windows running first and point the Linux side at the Windows machine IP instead of localhost.

---

## 3. Install the src Logger Dependencies

The combined logger lives in `src/teleop_recorder.py`. It depends on the RealMan SDK, the LEAP hand utilities bundled in this repository, OpenVR for tracker input, ZMQ for MANUS data, and HDF5 for logging.

### 3.1 Create a Python environment

```bash
cd /home/rmtest/FRIRobot
python3.9 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

### 3.2 Install Python packages

```bash
pip install \
  numpy \
  scipy \
  pyzmq \
  h5py \
  openvr \
  dynamixel_sdk \
  Robotic_Arm
```

Optional, only if you want to enable the camera logging path:

```bash
pip install pyrealsense2
```

Optional, only if you want the interactive debugger hook used by `src/track.py`:

```bash
pip install ipython
```

Notes:

- `Robotic_Arm` provides the RealMan Python interface used by the logger.
- `dynamixel_sdk` is required by the LEAP hand controller.
- The repository already includes `src/leap_hand_utils/`, so no extra install step is needed for that package.

### 3.3 Verify the Python dependencies

Run a quick import check before starting the logger:

```bash
python - <<'PY'
import numpy
import scipy
import zmq
import h5py
import openvr
from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e
print("Python dependency check passed")
PY
```

If `Robotic_Arm` is not available from pip in your environment, use the local SDK path instead:

```bash
export PYTHONPATH="/home/rmtest/FRIRobot/RMAPI/Python:$PYTHONPATH"
```

---

## 4. Run Order

The teleoperation stack should be started in this order:

1. Power on the hardware: RealMan arm, LEAP Hand, Vive Tracker, and MANUS gloves.
2. Start SteamVR and confirm the Vive Tracker is visible.
3. Start the MANUS SDK client and confirm it is publishing ergonomics data on `tcp://127.0.0.1:8000` or the configured host endpoint.
4. Run the combined logger from `src/`.

```bash
cd /home/rmtest/FRIRobot/src
python teleop_recorder.py
```

The logger will create two paired files under `src/logs/` by default:

- `teleop_data_N.hdf5`
- `teleop_metadata_N.txt` with recording date/time, control frequency, interpolation decay, and runtime statistics

If you pass `--log-path custom_name.hdf5`, the paired metadata file is written as `custom_name.txt` with the same compact fields.

---

## 5. Recommended Smoke Tests

Before a full run, these checks are useful:

```bash
# MANUS SDK build directory
cd /home/rmtest/FRIRobot/LMAPI/MANUS_Core_2.4.0_SDK/SDKClient_Linux
./SDKClient_Linux.out

# Python-side dependency check
cd /home/rmtest/FRIRobot/src
python teleop_recorder.py --help
```

If the logger help text appears without import errors, the Python environment is close to ready.

---

## 6. Common Issues

- `No module named Robotic_Arm`: install `Robotic_Arm` or add `RMAPI/Python` to `PYTHONPATH`.
- `h5py not installed`: install `h5py`; the logger can run without logging, but this project expects logging to be enabled.
- `No Vive Trackers found`: SteamVR is not running, the tracker is not paired, or the OpenVR runtime is not available.
- `Failed to connect to LEAP Hand`: check the USB serial device and try `--hand-port /dev/ttyUSB0` or `--hand-port /dev/ttyUSB1`.
- `Waiting for MANUS ergonomics data`: the MANUS SDK client is not running or the ZMQ endpoint does not match.
- `Failed to connect to RealMan arm`: verify the arm IP address and network connectivity.

---

## 7. What Depends On What

- `LMAPI/MANUS_Core_2.4.0_SDK/SDKClient_Linux/` provides the glove data stream consumed by the logger.
- `src/teleop_recorder.py` provides live arm control and HDF5 logging.
- `src/replay_hdf5.py` replays the saved logs created by the logger.

If you only need to record new teleoperation data, the two critical runtime pieces are the MANUS SDK client and the combined logger.
