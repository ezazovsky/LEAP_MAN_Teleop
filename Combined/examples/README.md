# Training Examples

This directory contains example scripts for training policies using the teleoperation data with integrated RealSense camera frames.

## Available Examples

### 1. Simple Imitation Learning (`train_imitation_policy.py`)

A complete example of training a visuomotor policy network using the HDF5 logged teleoperation data.

**What it does:**
- Loads HDF5 logs with RGB and depth frames
- Creates context windows of camera observations
- Trains a CNN+MLP network to predict arm end-effector poses from observations
- Validates on held-out data and saves the best model

**Usage:**

```bash
cd /home/fri/FRIRobot-main/Combined

# Train with default settings
python examples/train_imitation_policy.py logs/teleop_20260426_120000.hdf5

# Train with custom hyperparameters
python examples/train_imitation_policy.py logs/teleop_20260426_120000.hdf5 \
    --context-length 10 \
    --batch-size 64 \
    --epochs 100 \
    --learning-rate 1e-4 \
    --val-split 0.2 \
    --output-dir ./my_models
```

**Command-line options:**

```
  --context-length      Number of past frames for context (default: 5)
  --batch-size          Batch size for training (default: 32)
  --epochs              Number of training epochs (default: 50)
  --learning-rate       Learning rate (default: 1e-3)
  --val-split           Validation split fraction (default: 0.2)
  --output-dir          Directory to save model and logs (default: ./model_outputs)
```

**Expected output:**

```
Using device: cuda
Loading dataset...
  Training samples: 8000
  Validation samples: 2000

Initializing model...
  Total parameters: 1,234,567

Training...

Epoch 1/50
  Batch 100/250: Loss = 0.3542
  Batch 200/250: Loss = 0.2187
  Train loss: 0.2456
  Val loss: 0.1832, Pose error: 0.0234
  Saved best model to ./model_outputs/best_model.pth

...
```

## Requirements

Install additional dependencies:

```bash
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install Pillow tensorboard
```

## Data Format for Training

The scripts expect HDF5 files created with `--enable-realsense` flag:

```bash
python combined_simple_teleop_real_logger.py --enable-realsense
```

This creates logs with:
- RGB camera frames (640×480 default)
- Depth frames (16-bit, mm units)
- Arm poses (6D: x, y, z, rx, ry, rz)
- Hand joint angles (20D)
- Hand pose (16D retargeted LEAP angles)
- Synchronized timestamps

## Model Architecture

The imitation policy uses a simple architecture suitable for beginner-intermediate learning:

```
Input RGB [3, 480, 640]
   ↓
Conv2d(3 → 32, kernel=3, stride=2)
ReLU
Conv2d(32 → 64, kernel=3, stride=2)
ReLU
AdaptiveAvgPool2d → 8×8
   ↓
Flatten [64×8×8] + Hand State [20]
   ↓
Dense(4116 → 256) + ReLU + Dropout
Dense(256 → 128) + ReLU + Dropout
Dense(128 → 6)  [output: arm pose]
   ↓
Output Pose [6]
```

**Total parameters:** ~1.2M (very fast to train, suitable for single GPU)

## Extending the Examples

### Adding Depth Input

Modify `train_imitation_policy.py` to use depth frames:

```python
# In forward pass
def forward(self, rgb, depth, hand_state):
    rgb_features = self.conv_rgb(rgb)
    depth_features = self.conv_depth(depth)
    combined = torch.cat([rgb_features, depth_features, hand_state], dim=1)
    return self.mlp(combined)
```

### Multi-action Prediction

Predict sequences of actions instead of single steps:

```python
# In model output
output_dim = 6 * action_sequence_length  # Predict 5 steps ahead
```

### Adding Hand Configuration to Output

Jointly predict arm and hand commands:

```python
output_dim = 6 + 16  # 6D arm pose + 16D hand pose
```

## Troubleshooting

### CUDA Out of Memory

```bash
# Reduce batch size
python examples/train_imitation_policy.py logs/teleop.hdf5 --batch-size 16

# Use CPU (slower)
# Modify script to add: device = torch.device('cpu')
```

### Validation Loss Not Improving

1. Increase context length: `--context-length 10`
2. Increase learning rate: `--learning-rate 1e-2`
3. Check data quality with `replay_with_camera.py --summary logs/teleop.hdf5`
4. Ensure teleop was smooth (no CANFD errors)

### Poor Policy Performance After Training

- More data: Collect longer teleoperation runs
- Normalize observations better (currently just 0-255 → 0-1)
- Use regularization: Add L2 penalty to model weights
- Data augmentation: Random crops, brightness adjustments to images

## Next Steps

After training a policy:

1. **Save model weights** (automatically done in `./model_outputs/best_model.pth`)
2. **Evaluate on test set** with proper metrics
3. **Deploy to real robot** for closed-loop control
4. **Fine-tune with reinforcement learning** for online adaptation
5. **Collect more diverse data** to improve generalization

## References

- `replay_with_camera.py`: Data loading utilities
- `REALSENSE_CAMERA_INTEGRATION.md`: Details on camera data format
- `DATA_PIPELINE.md`: Teleoperation pipeline documentation
