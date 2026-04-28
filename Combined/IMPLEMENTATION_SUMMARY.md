# HDF5 Numbered Naming & Metadata File Implementation

## Summary of Changes

This implementation adds **numbered file naming** and **metadata logging** to the teleop recording system.

### 1. Numbered File Naming

#### **Before:**
```
Combined/logs/teleop_20260428_142525.hdf5
```

#### **After:**
```
Combined/logs/teleop_data_0.hdf5
Combined/logs/teleop_data_1.hdf5
Combined/logs/teleop_data_2.hdf5
...
```

### 2. Automatic Metadata Files

For each HDF5 recording, a corresponding metadata text file is automatically created:

```
Combined/logs/teleop_metadata_0.txt
Combined/logs/teleop_metadata_1.txt
Combined/logs/teleop_metadata_2.txt
...
```

### 3. Metadata File Contents

Each metadata file contains:

```
================== Teleop Recording Metadata ==================
Recording Date (UTC): 2026-04-28T14:25:25.123456Z
Recording Date (Local): 2026-04-28T14:25:25.123456

--- Control Parameters ---
Control Frequency (Hz): 125.0
Interpolation Decay (Position): 0.15
Interpolation Decay (Rotation): 0.15

--- Recording Statistics ---
Total Runtime (seconds): 45.23
Total Samples: 5654
Average Frequency (Hz): 125.03

--- Robot Configuration ---
Robot IP: 192.168.1.18
Robot Port: 8080
Arm Position Scale: 1.0
Arm Rotation Scale: 1.0

--- Hardware Enabled ---
Camera Enabled: False
Hand Current Limit (mA): 350
Hand Side: right

--- File Information ---
HDF5 Data File: teleop_data_0.hdf5
Metadata File: teleop_metadata_0.txt
============================================================
```

## Technical Implementation Details

### Changes to `combined_simple_teleop_real_logger.py`

#### 1. **New Helper Function**
```python
def get_next_numbered_filename(log_dir, prefix, suffix)
```
- Automatically finds the next available number for files
- Creates logs directory if it doesn't exist
- Example: `get_next_numbered_filename(log_dir, "teleop_data", ".hdf5")` returns `("teleop_data_0.hdf5", 0)` and then `("teleop_data_1.hdf5", 1)` on next call, etc.

#### 2. **Updated HDF5LoggingProcess**

**Constructor Changes:**
- Added 3 new parameters: `metadata_path`, `metadata`, and removed timestamp-based naming
- Stores `start_time` to calculate total runtime

**New Method:**
```python
def _write_metadata_file(self, sample_count, total_time)
```
- Writes metadata to text file at process shutdown
- Includes timing statistics calculated from the recording session

**Enhanced Cleanup:**
- Calculates `total_time` and adds it to HDF5 file attributes
- Calls `_write_metadata_file()` before process exits

#### 3. **Updated Setup Method**

The `setup()` method now:
- Generates numbered filenames using `get_next_numbered_filename()`
- Creates metadata dictionary with:
  - Recording timestamps (UTC and local)
  - Interpolation parameters (alpha_pos, alpha_rot)
  - Scaling factors (arm_pos_scale, arm_rot_scale)
- Passes metadata to HDF5LoggingProcess
- Prints both the HDF5 and metadata file paths for user reference

#### 4. **Backwards Compatibility**

If custom `--log-path` is provided, the script:
- Uses the custom path as-is for the HDF5 file
- Automatically generates a matching `.txt` file in the same location

## Usage

### Standard Usage (Numbered Files)
```bash
python combined_simple_teleop_real_logger.py
# Creates: teleop_data_0.hdf5 and teleop_metadata_0.txt
# On next run: teleop_data_1.hdf5 and teleop_metadata_1.txt
```

### Custom Path Support
```bash
python combined_simple_teleop_real_logger.py --log-path /custom/path/my_recording.hdf5
# Creates: my_recording.hdf5 and my_recording.txt
```

## Benefits

1. **Organization**: Files are sequentially numbered, making it easy to find and reference recordings
2. **Debugging**: Metadata files provide complete context about each recording session
3. **Automation**: No need to manually track recording parameters or creation times
4. **Integration**: Metadata can be parsed by post-processing scripts for analysis
5. **Traceability**: Total runtime and sample count allow verification of complete recordings

## Files Modified

- `Combined/combined_simple_teleop_real_logger.py` - Main implementation file
