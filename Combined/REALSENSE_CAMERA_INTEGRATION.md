# RealSense D435i Camera Integration for Teleoperation

This document describes the integration of the Intel RealSense D435i depth camera into the teleoperation pipeline for easy policy learning and training.

## Overview

The `combined_simple_teleop_real_logger.py` script now captures RealSense D435i RGB and depth frames alongside all teleoperation data (arm poses, hand joints, glove data). All data is stored in a single HDF5 file with frame-accurate synchronization, making it trivial to use for imitation learning.

### What's Captured

Per teleoperation timestep:
- **Arm control**: Raw, bounded, safe, and smoothed end-effector poses (6D: x,y,z,rx,ry,rz)
- **Hand control**: MANUS glove joint angles (20D) and LEAP Hand retargeted poses (16D)
- **Camera**: Synchronized RGB (BGR, 8-bit) and depth (16-bit millimeters) frames
- **Metadata**: Frame indices, timestamps, hold flags, CANFD status
- **Time synchronization**: Wall-clock time, monotonic perf counter time

## Installation

### 1. Install RealSense SDK and Python Bindings

```bash
# On Ubuntu/Debian:
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6270CDCC
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt install librealsense2-dkms librealsense2-utils

# On macOS (using Homebrew):
brew install librealsense2

# Install Python bindings
pip install pyrealsense2

# (Optional) For image export: pip install Pillow
```

### 2. Verify Camera Connection

```bash
# List connected RealSense devices
realsense-viewer

# Or check via Python:
python -c "import pyrealsense2 as rs; ctx = rs.context(); print([d.get_info(rs.camera_info.name) for d in ctx.query_devices()])"
```

## Usage

### Basic Teleoperation with Camera

```bash
cd /home/fri/FRIRobot-main/Combined

# Start with RealSense camera at 30 FPS (default 640x480)
python combined_simple_teleop_real_logger.py \
    --enable-realsense \
    --realsense-fps 30 \
    --realsense-width 640 \
    --realsense-height 480
```

### Advanced Configuration

```bash
# Higher resolution depth + RGB, lower FPS to avoid CPU overload
python combined_simple_teleop_real_logger.py \
    --enable-realsense \
    --realsense-width 1280 \
    --realsense-height 720 \
    --realsense-fps 15 \
    --control-hz 125.0

# Depth-only capture (faster, uses less storage):
python combined_simple_teleop_real_logger.py \
    --enable-realsense \
    --no-realsense-rgb

# RGB-only capture (e.g., for visual policy only):
python combined_simple_teleop_real_logger.py \
    --enable-realsense \
    --no-realsense-depth

# Custom log path:
python combined_simple_teleop_real_logger.py \
    --enable-realsense \
    --log-path ./my_dataset/teleop_run_001.hdf5
```

### Command-Line Arguments Reference

```
RealSense Options:
  --enable-realsense          Enable D435i camera capture
  --realsense-width           Frame width in pixels (default: 640)
  --realsense-height          Frame height in pixels (default: 480)
  --realsense-fps             Capture framerate (default: 30)
  --realsense-rgb             Enable RGB capture (default: enabled)
  --no-realsense-rgb          Disable RGB capture
  --realsense-depth           Enable depth capture (default: enabled)
  --no-realsense-depth        Disable depth capture

Other Options:
  --log-flush-every           HDF5 flush frequency (samples, default: 50)
  --control-hz                Teleop control rate (default: 125.0)
```

## Data Format

### HDF5 Structure

```
/
├── time/
│   ├── monotonic_s          [N] float64  - perf_counter() timestamps
│   └── wall_time_s          [N] float64  - time.time() timestamps
│
├── arm/
│   ├── raw_pose             [N, 6] float64  - Unfiltered VR tracker poses
│   ├── bounded_pose         [N, 6] float64  - Safety-bounded poses
│   ├── safe_pose            [N, 6] float64  - Holding poses when bounded
│   ├── smoothed_pose        [N, 6] float64  - Commands sent to robot arm
│   ├── hold_flag            [N] bool        - True when safety limit active
│   └── canfd_status         [N] int32       - Robot communication status (0=ok)
│
├── hand/
│   ├── manus_joints         [N, 20] float64 - All glove joint angles
│   ├── leap_pose            [N, 16] float64 - Retargeted LEAP Hand angles
│   └── has_glove_data       [N] bool        - Whether glove data available
│
└── camera/                             (only if --enable-realsense)
    ├── rgb                  [N] uint8_vlen  - BGR frames (H×W×3), flattened
    ├── depth                [N] uint16_vlen - Depth frames (H×W), flattened, mm units
    ├── frame_time           [N] float64     - Camera frame timestamps (ms)
    └── frame_index          [N] int64       - Unique frame ID (-1 if dropped)
```

### HDF5 File Attributes

```python
f.attrs['camera_enabled']    - bool, True if camera was used
f.attrs['camera_width']      - int, frame width in pixels
f.attrs['camera_height']     - int, frame height in pixels
f.attrs['camera_fps']        - int, camera framerate
f.attrs['robot_ip']          - str, robot IP address
f.attrs['robot_port']        - int, robot port
f.attrs['control_hz']        - float, control loop rate
f.attrs['created_utc']       - str, ISO8601 timestamp
f.attrs['sample_count']      - int, total samples
```

## Loading and Processing Data

### Python: Policy Training Example

```python
import h5py
import numpy as np
from replay_with_camera import HDF5CameraDataLoader

# Load log file
loader = HDF5CameraDataLoader(
    "logs/teleop_20260426_120000.hdf5",
    load_images=True
)

# Print summary
loader.print_summary()

# Get single sample with all data
sample = loader.get_sample(0)
rgb_frame = sample['camera/rgb']        # Shape: (480, 640, 3), uint8 BGR
depth_frame = sample['camera/depth']    # Shape: (480, 640), uint16 mm
arm_pose = sample['arm/smoothed_pose']  # Shape: (6,), float64
hand_joints = sample['hand/manus_joints']  # Shape: (20,), float64

# Load continuous trajectory for sequence learning
trajectory = loader.get_trajectory(start_idx=0, end_idx=1000)
# trajectory['camera/rgb']        - list of RGB frames or array
# trajectory['camera/depth']      - list of depth frames or array
# trajectory['arm/smoothed_pose'] - [1001, 6] array
# trajectory['hand/manus_joints'] - [1001, 20] array

# Export images for visualization
loader.export_images(
    output_dir="./exported_frames",
    indices=range(0, 100, 10)  # Every 10th frame
)
```

### PyTorch DataLoader

```python
import torch
from torch.utils.data import Dataset, DataLoader

class TeleoperationDataset(Dataset):
    def __init__(self, hdf5_path, context_length=10):
        """
        context_length: Number of past frames for recurrent networks
        """
        self.loader = HDF5CameraDataLoader(hdf5_path)
        self.context_length = context_length
        self.length = max(0, len(self.loader) - context_length)
        
    def __len__(self):
        return self.length
    
    def __getitem__(self, idx):
        # Get context window
        trajectory = self.loader.get_trajectory(idx, idx + self.context_length)
        
        # Normalize images
        rgb_frames = np.stack([f for f in trajectory['camera/rgb'] if f is not None])
        rgb_normalized = rgb_frames.astype(np.float32) / 255.0
        rgb_tensor = torch.from_numpy(rgb_normalized).permute(0, 3, 1, 2)  # [T, C, H, W]
        
        # Get action (next pose target)
        next_pose = self.loader.get_sample(idx + self.context_length)['arm/smoothed_pose']
        action = torch.from_numpy(next_pose).float()
        
        # Get state (current hand configuration)
        state = torch.from_numpy(trajectory['hand/manus_joints'][-1]).float()
        
        return {
            'images': rgb_tensor,
            'state': state,
            'action': action  # Target arm pose
        }

# Use dataset
dataset = TeleoperationDataset("logs/teleop_20260426_120000.hdf5", context_length=5)
loader = DataLoader(dataset, batch_size=32, shuffle=True, num_workers=4)

for batch in loader:
    images = batch['images']           # [B, T, C, H, W]
    states = batch['state']            # [B, 20]
    actions = batch['action']          # [B, 6]
    # Train your model...
```

### Command-Line Tools

```bash
# Print summary of log file
python replay_with_camera.py logs/teleop_20260426_120000.hdf5 --summary

# Inspect frame at index 100
python replay_with_camera.py logs/teleop_20260426_120000.hdf5 --sample 100

# Export RGB/depth frames as PNG
python replay_with_camera.py logs/teleop_20260426_120000.hdf5 \
    --export-images ./exported_frames \
    --export-indices 0 50 100 200

# Load trajectory for analysis
python replay_with_camera.py logs/teleop_20260426_120000.hdf5 \
    --trajectory 0 500
```

## Storage Considerations

### Typical Data Sizes

For a 5-minute teleoperation run at 125 Hz with 30 FPS camera:

| Resolution | RGB+Depth | RGB Only | Depth Only |
|------------|-----------|----------|-----------|
| 640×480    | ~1.2 GB   | ~400 MB  | ~800 MB    |
| 1280×720   | ~3.0 GB   | ~900 MB  | ~2.1 GB    |

**HDF5 compression** (`h5py` automatic):
- Typical compression ratio: 2:1 to 3:1 for depth
- RGB is harder to compress (~1.1:1)

### Storage Tips

1. **Use lower resolution for fast iteration**: Start with 640×480@15fps during development
2. **Depth-only captures**: If only depth is needed, use `--no-realsense-rgb` to cut storage by 50%
3. **Periodic flushing**: Adjust `--log-flush-every` to balance safety vs. performance
   - Higher values = faster logging but risk data loss on crash
   - Lower values = safer but slight performance hit

## Troubleshooting

### Camera Not Found

```bash
# Check USB permissions
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER

# Restart udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Slow Capture / Frame Drops

- Reduce resolution (`--realsense-width 640 --realsense-height 480`)
- Reduce FPS (`--realsense-fps 15`)
- Disable RGB if only depth needed (`--no-realsense-rgb`)
- Check CPU load: `top` or `htop`

### Missing Frames in HDF5

- Check `camera/frame_index` dataset for -1 values (indicates drops)
- May indicate USB bandwidth saturation
- Try different USB port or USB 3.0

### HDF5 File Corruption

- Always press Ctrl+C gracefully to flush data
- Check with: `h5py.File(path, 'r').keys()`

## Frame Synchronization Details

### Timing Strategy

The camera runs in a **separate background thread** with its own capture loop:

1. **Main teleop loop**: 125 Hz deterministic (8 ms cycles)
2. **Camera thread**: Independent, captures at specified FPS (30 FPS = 33.3 ms)
3. **Frame association**: Each teleoperation sample captures the **latest available camera frame** at logging time

### Timestamps

Three timestamps per sample:

```python
# High-resolution teleoperation timing
sample['time/monotonic_s']   # perf_counter(), for relative timing
sample['time/wall_time_s']   # time.time(), for external synchronization

# Camera frame timing (from RealSense hardware)
sample['camera/frame_time']  # RealSense internal timestamp
sample['camera/frame_index'] # Unique frame ID to detect drops
```

### For Trajectory Learning

Use the **monotonic timestamps** to compute inter-frame deltas:

```python
# Get action frequency information
dt = trajectory['time/monotonic_s'][1] - trajectory['time/monotonic_s'][0]
action_hz = 1.0 / np.mean(np.diff(trajectory['time/monotonic_s']))
print(f"Average action frequency: {action_hz:.1f} Hz")
```

## Best Practices

1. **Always start with a short calibration run** to verify everything works
2. **Pre-create a `logs/` directory** to avoid permission issues
3. **Monitor CPU/disk during first runs** - high-res depth at high FPS can be demanding
4. **Use timestamps, not indices** for temporal correlations in training
5. **Check frame_index for monotonicity** to detect dropped frames
6. **Normalize images** to [0, 1] or [-1, 1] before feeding to ML models

## Example: Training a Simple Visuomotor Policy

See `Combined/examples/train_imitation_policy.py` for a complete example using the camera data.

---

**Questions?** Check `DOCUMENTATION_INDEX.txt` or the original `DATA_PIPELINE.md` for related documentation.
