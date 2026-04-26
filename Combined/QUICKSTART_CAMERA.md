# Quick Reference: RealSense Integration

## Installation (one-time)

```bash
# Install RealSense SDK and Python bindings
pip install pyrealsense2 h5py

# Optional: For image export
pip install Pillow

# Optional: For training examples
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

## 📹 Capture Teleoperation Data with Camera

```bash
cd Combined

# Basic: 640×480 RGB+Depth at 30 FPS
python combined_simple_teleop_real_logger.py --enable-realsense

# High-resolution depth (be careful with USB bandwidth)
python combined_simple_teleop_real_logger.py \
    --enable-realsense \
    --realsense-width 1280 \
    --realsense-height 720 \
    --realsense-fps 15

# Depth-only (smaller files, faster I/O)
python combined_simple_teleop_real_logger.py \
    --enable-realsense \
    --no-realsense-rgb

# RGB-only
python combined_simple_teleop_real_logger.py \
    --enable-realsense \
    --no-realsense-depth
```

## 🔍 Inspect Captured Data

```bash
# View file summary
python replay_with_camera.py logs/teleop_*.hdf5 --summary

# Inspect single frame at index 100
python replay_with_camera.py logs/teleop_*.hdf5 --sample 100

# Export frames 0-500 every 10th frame as PNG
python replay_with_camera.py logs/teleop_*.hdf5 \
    --export-images ./frames \
    --export-indices 0 10 20 30 ...

# Load and analyze trajectory (0-1000 samples)
python replay_with_camera.py logs/teleop_*.hdf5 \
    --trajectory 0 1000
```

## 🤖 Train Imitation Learning Policy

```bash
# Basic training with default settings
python examples/train_imitation_policy.py logs/teleop_*.hdf5

# Custom training
python examples/train_imitation_policy.py logs/teleop_*.hdf5 \
    --context-length 10 \
    --batch-size 64 \
    --epochs 100 \
    --learning-rate 1e-4 \
    --output-dir ./models_v1
```

## 📊 Load Data in Python (for custom training)

```python
from replay_with_camera import HDF5CameraDataLoader
import torch
from torch.utils.data import DataLoader

# Load dataset
loader = HDF5CameraDataLoader("logs/teleop_*.hdf5")

# Get single sample
sample = loader.get_sample(0)
rgb = sample['camera/rgb']              # [480, 640, 3] uint8 BGR
depth = sample['camera/depth']          # [480, 640] uint16 mm
pose = sample['arm/smoothed_pose']      # [6] float64
hand = sample['hand/manus_joints']      # [20] float64

# Load full trajectory
traj = loader.get_trajectory(start_idx=0, end_idx=1000)
# traj['camera/rgb'] → list of frames
# traj['camera/depth'] → list of depth
# traj['arm/smoothed_pose'] → [1001, 6] array

# Export images
loader.export_images(output_dir="./exported", indices=[0, 100, 200])
```

## 🎯 Data Format for Training

Each HDF5 file contains:

```python
sample = loader.get_sample(i)

# Timestamps
sample['time/monotonic_s']        # perf_counter() - for inter-frame timing
sample['time/wall_time_s']        # time.time() - for external sync

# Camera (if captured)
sample['camera/rgb']              # [H, W, 3] uint8, BGR format
sample['camera/depth']            # [H, W] uint16, millimeters
sample['camera/frame_time']       # float64, hardware timestamp
sample['camera/frame_index']      # int64, frame ID (-1 if dropped)

# Arm end-effector pose (6D: x,y,z,rx,ry,rz in radians)
sample['arm/smoothed_pose']       # [6] - actual command to robot
sample['arm/raw_pose']            # [6] - unfiltered VR input
sample['arm/bounded_pose']        # [6] - after safety bounds
sample['arm/hold_flag']           # bool - True if held by safety
sample['arm/canfd_status']        # int32 - communication status (0=ok)

# Hand configuration
sample['hand/manus_joints']       # [20] - glove joint angles
sample['hand/leap_pose']          # [16] - retargeted LEAP hand
sample['hand/has_glove_data']     # bool - True if glove available
```

## ⚙️ Configuration Options

```bash
# Teleoperation options
--control-hz 125.0                    # Control loop rate (don't change)
--calibration-countdown 3             # Seconds before starting
--arm-pos-scale 1.0                   # Position scaling
--arm-rot-scale 1.0                   # Rotation scaling

# Logging options
--log-hdf5                            # Enable logging (default: True)
--log-path ./my_logs/run_001.hdf5     # Custom log path
--log-flush-every 50                  # HDF5 flush frequency

# RealSense camera options
--enable-realsense                    # Enable camera
--realsense-width 640                 # Frame width (pixels)
--realsense-height 480                # Frame height (pixels)
--realsense-fps 30                    # Capture framerate (FPS)
--realsense-rgb                       # Enable RGB (default)
--no-realsense-rgb                    # Disable RGB
--realsense-depth                     # Enable depth (default)
--no-realsense-depth                  # Disable depth
```

## 📈 File Sizes (5-minute capture)

| Resolution | Config      | Size  |
|-----------|------------|-------|
| 640×480   | RGB+Depth  | 1.2GB |
| 640×480   | RGB only   | 400MB |
| 640×480   | Depth only | 800MB |
| 1280×720  | RGB+Depth  | 3.0GB |

*Note: Actual sizes vary by motion complexity and HDF5 compression (~2:1 ratio)*

## 🔧 Troubleshooting

| Problem | Solution |
|---------|----------|
| Camera not found | Check USB connection, run `realsense-viewer` |
| Frame drops | Reduce resolution or FPS |
| Slow logging | Reduce `--log-flush-every` or decrease resolution |
| Training loss not improving | More data, longer context-length, better preprocessing |
| CUDA out of memory | Reduce `--batch-size` or use CPU |
| HDF5 file corrupt | Always Ctrl+C gracefully to flush data |

## 📚 Documentation Files

- **REALSENSE_CAMERA_INTEGRATION.md** - Complete integration guide
- **examples/README.md** - Training examples and extensions
- **replay_with_camera.py** - Data loading utilities (documented inline)
- **combined_simple_teleop_real_logger.py** - Main capture script

## 🚀 Typical Workflow

1. **Collect data**
   ```bash
   python combined_simple_teleop_real_logger.py --enable-realsense
   ```

2. **Inspect and validate**
   ```bash
   python replay_with_camera.py logs/teleop_*.hdf5 --summary
   python replay_with_camera.py logs/teleop_*.hdf5 --export-images ./check_frames
   ```

3. **Train policy**
   ```bash
   python examples/train_imitation_policy.py logs/teleop_*.hdf5 --epochs 50
   ```

4. **Evaluate and deploy**
   - Load saved model from `./model_outputs/best_model.pth`
   - Integrate with robot control loop
   - Test in sim before real robot deployment

---

**Questions?** See `REALSENSE_CAMERA_INTEGRATION.md` for detailed documentation.
