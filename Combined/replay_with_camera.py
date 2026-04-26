"""
Replay and analyze HDF5 teleoperation logs with integrated RealSense camera data.

This script loads HDF5 logs captured with combined_simple_teleop_real_logger.py,
reconstructs RGB and depth frames, and provides utilities for:
- Visualizing teleoperation sequences
- Extracting synchronized state-action pairs for policy training
- Verifying frame synchronization and data integrity
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import h5py


class HDF5CameraDataLoader:
    """
    Load and process HDF5 teleoperation logs with synchronized camera frames.
    Designed for easy integration with PyTorch DataLoader for imitation learning.
    """

    def __init__(self, hdf5_path, load_images=True, image_output_dir=None):
        """
        Initialize data loader from HDF5 file.

        Args:
            hdf5_path: Path to HDF5 teleoperation log.
            load_images: Whether to reconstruct and load images into memory.
            image_output_dir: Optional directory to save reconstructed images as PNG.
        """
        self.hdf5_path = hdf5_path
        self.load_images = load_images
        self.image_output_dir = image_output_dir

        if image_output_dir:
            os.makedirs(image_output_dir, exist_ok=True)

        # Load metadata
        with h5py.File(self.hdf5_path, "r") as f:
            self.sample_count = f.attrs.get("sample_count", 0)
            self.has_camera = f.attrs.get("camera_enabled", False)
            self.camera_width = f.attrs.get("camera_width", 640)
            self.camera_height = f.attrs.get("camera_height", 480)

            # Verify camera datasets if camera was enabled
            if self.has_camera:
                assert "camera/rgb" in f or "camera/depth" in f, (
                    "Camera enabled but no image datasets found"
                )

        print(f"Loaded HDF5 log: {self.hdf5_path}")
        print(f"  Samples: {self.sample_count}")
        print(f"  Camera enabled: {self.has_camera}")
        if self.has_camera:
            print(f"  Camera resolution: {self.camera_width}x{self.camera_height}")

    def __len__(self):
        return self.sample_count

    def get_sample(self, index):
        """
        Get a single sample with all synchronized data.

        Returns:
            dict with keys:
                - time/ (monotonic_s, wall_time_s)
                - arm/ (raw_pose, bounded_pose, safe_pose, smoothed_pose, hold_flag, canfd_status)
                - hand/ (manus_joints, leap_pose, has_glove_data)
                - camera/ (rgb, depth, frame_time, frame_index) [if camera enabled]
        """
        with h5py.File(self.hdf5_path, "r") as f:
            sample = {}

            # Time data
            sample["time/monotonic_s"] = float(f["time/monotonic_s"][index])
            sample["time/wall_time_s"] = float(f["time/wall_time_s"][index])

            # Arm data
            sample["arm/raw_pose"] = np.array(f["arm/raw_pose"][index])
            sample["arm/bounded_pose"] = np.array(f["arm/bounded_pose"][index])
            sample["arm/safe_pose"] = np.array(f["arm/safe_pose"][index])
            sample["arm/smoothed_pose"] = np.array(f["arm/smoothed_pose"][index])
            sample["arm/hold_flag"] = bool(f["arm/hold_flag"][index])
            sample["arm/canfd_status"] = int(f["arm/canfd_status"][index])

            # Hand data
            sample["hand/manus_joints"] = np.array(f["hand/manus_joints"][index])
            sample["hand/leap_pose"] = np.array(f["hand/leap_pose"][index])
            sample["hand/has_glove_data"] = bool(f["hand/has_glove_data"][index])

            # Camera data
            if self.has_camera:
                rgb_flat = np.array(f["camera/rgb"][index], dtype=np.uint8)
                depth_flat = np.array(f["camera/depth"][index], dtype=np.uint16)

                # Reconstruct images
                if len(rgb_flat) > 0:
                    rgb = rgb_flat.reshape(
                        (self.camera_height, self.camera_width, 3)
                    )
                    sample["camera/rgb"] = rgb
                else:
                    sample["camera/rgb"] = None

                if len(depth_flat) > 0:
                    depth = depth_flat.reshape(
                        (self.camera_height, self.camera_width)
                    )
                    sample["camera/depth"] = depth
                else:
                    sample["camera/depth"] = None

                sample["camera/frame_time"] = float(f["camera/frame_time"][index])
                sample["camera/frame_index"] = int(f["camera/frame_index"][index])

            return sample

    def get_trajectory(self, start_idx=0, end_idx=None):
        """
        Get a continuous trajectory of samples for sequence-based learning.

        Args:
            start_idx: Starting sample index.
            end_idx: Ending sample index (inclusive). Defaults to end of file.

        Returns:
            dict with batched data, suitable for batch processing.
        """
        if end_idx is None:
            end_idx = self.sample_count - 1

        trajectory = {
            "time/monotonic_s": [],
            "time/wall_time_s": [],
            "arm/smoothed_pose": [],
            "arm/hold_flag": [],
            "hand/manus_joints": [],
            "hand/leap_pose": [],
            "hand/has_glove_data": [],
        }

        if self.has_camera:
            trajectory["camera/rgb"] = []
            trajectory["camera/depth"] = []

        with h5py.File(self.hdf5_path, "r") as f:
            indices = np.arange(start_idx, min(end_idx + 1, self.sample_count))

            for idx in indices:
                trajectory["time/monotonic_s"].append(
                    float(f["time/monotonic_s"][idx])
                )
                trajectory["time/wall_time_s"].append(
                    float(f["time/wall_time_s"][idx])
                )
                trajectory["arm/smoothed_pose"].append(
                    np.array(f["arm/smoothed_pose"][idx])
                )
                trajectory["arm/hold_flag"].append(
                    bool(f["arm/hold_flag"][idx])
                )
                trajectory["hand/manus_joints"].append(
                    np.array(f["hand/manus_joints"][idx])
                )
                trajectory["hand/leap_pose"].append(
                    np.array(f["hand/leap_pose"][idx])
                )
                trajectory["hand/has_glove_data"].append(
                    bool(f["hand/has_glove_data"][idx])
                )

                if self.has_camera:
                    rgb_flat = np.array(f["camera/rgb"][idx], dtype=np.uint8)
                    depth_flat = np.array(f["camera/depth"][idx], dtype=np.uint16)

                    if len(rgb_flat) > 0:
                        rgb = rgb_flat.reshape(
                            (self.camera_height, self.camera_width, 3)
                        )
                    else:
                        rgb = None
                    trajectory["camera/rgb"].append(rgb)

                    if len(depth_flat) > 0:
                        depth = depth_flat.reshape(
                            (self.camera_height, self.camera_width)
                        )
                    else:
                        depth = None
                    trajectory["camera/depth"].append(depth)

        # Convert lists to arrays where appropriate
        trajectory["time/monotonic_s"] = np.array(
            trajectory["time/monotonic_s"]
        )
        trajectory["time/wall_time_s"] = np.array(trajectory["time/wall_time_s"])
        trajectory["arm/smoothed_pose"] = np.array(
            trajectory["arm/smoothed_pose"]
        )
        trajectory["arm/hold_flag"] = np.array(trajectory["arm/hold_flag"])
        trajectory["hand/manus_joints"] = np.array(
            trajectory["hand/manus_joints"]
        )
        trajectory["hand/leap_pose"] = np.array(trajectory["hand/leap_pose"])
        trajectory["hand/has_glove_data"] = np.array(
            trajectory["hand/has_glove_data"]
        )

        print(
            f"Loaded trajectory: samples {start_idx}-{end_idx} "
            f"({len(indices)} frames, {indices[-1] - indices[0]:.2f}s window)"
        )

        return trajectory

    def export_images(self, output_dir=None, indices=None, format="png"):
        """
        Export RGB and depth frames as image files for visualization.

        Args:
            output_dir: Directory to save images. Uses default if None.
            indices: List of sample indices to export. Exports all if None.
            format: Image format ('png' or 'jpg').
        """
        if output_dir is None:
            output_dir = self.image_output_dir

        if not self.has_camera:
            print("No camera data in this log file.")
            return

        if output_dir is None:
            output_dir = "./exported_images"

        os.makedirs(output_dir, exist_ok=True)

        if indices is None:
            indices = range(self.sample_count)

        try:
            from PIL import Image
        except ImportError:
            print(
                "PIL/Pillow not installed. Install with: pip install Pillow"
            )
            return

        for idx in indices:
            sample = self.get_sample(idx)

            # Export RGB
            if sample.get("camera/rgb") is not None:
                rgb = sample["camera/rgb"]
                rgb_pil = Image.fromarray(rgb, mode="RGB")
                rgb_path = os.path.join(output_dir, f"rgb_{idx:06d}.{format}")
                rgb_pil.save(rgb_path)

            # Export depth as grayscale
            if sample.get("camera/depth") is not None:
                depth = sample["camera/depth"]
                # Normalize depth to 0-255 range for visualization
                depth_min = np.nanmin(depth[depth > 0])
                depth_max = np.nanmax(depth)
                if depth_max > depth_min:
                    depth_normalized = (
                        (depth - depth_min) / (depth_max - depth_min) * 255
                    ).astype(np.uint8)
                else:
                    depth_normalized = np.zeros_like(depth, dtype=np.uint8)

                depth_pil = Image.fromarray(depth_normalized, mode="L")
                depth_path = os.path.join(
                    output_dir, f"depth_{idx:06d}.{format}"
                )
                depth_pil.save(depth_path)

        print(
            f"Exported {len(indices)} image pairs to {output_dir}"
        )

    def print_summary(self):
        """Print detailed summary of log contents."""
        print("\n" + "=" * 60)
        print(f"HDF5 Log Summary: {self.hdf5_path}")
        print("=" * 60)

        with h5py.File(self.hdf5_path, "r") as f:
            print("\nFile Attributes:")
            for key, value in f.attrs.items():
                print(f"  {key}: {value}")

            print("\nDatasets:")
            for key in f.keys():
                ds = f[key]
                print(
                    f"  {key}: shape={ds.shape}, dtype={ds.dtype}"
                )

            print("\nSample Statistics:")
            if "arm/smoothed_pose" in f:
                poses = np.array(f["arm/smoothed_pose"])
                print(f"  Arm poses (6D): min={poses.min(axis=0)}, "
                      f"max={poses.max(axis=0)}")

            if "hand/manus_joints" in f:
                joints = np.array(f["hand/manus_joints"])
                valid_joints = joints[~np.isnan(joints)]
                if len(valid_joints) > 0:
                    print(f"  Hand joints (20D): min={valid_joints.min():.2f}, "
                          f"max={valid_joints.max():.2f}, "
                          f"nan_rate={(np.isnan(joints).sum() / joints.size * 100):.1f}%")

            if self.has_camera:
                if "camera/frame_index" in f:
                    frame_indices = np.array(f["camera/frame_index"])
                    valid_frames = frame_indices[frame_indices >= 0]
                    if len(valid_frames) > 0:
                        print(f"  Camera frames captured: {len(valid_frames)}/{self.sample_count}")
                        print(f"  Camera resolution: {self.camera_width}x{self.camera_height}")

        print("=" * 60 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Replay and analyze HDF5 teleoperation logs with camera data."
    )
    parser.add_argument(
        "hdf5_path",
        help="Path to HDF5 teleoperation log file.",
    )
    parser.add_argument(
        "--summary", action="store_true",
        help="Print summary of log contents and exit.",
    )
    parser.add_argument(
        "--export-images", type=str, default=None,
        help="Export RGB and depth frames to specified directory.",
    )
    parser.add_argument(
        "--export-indices", type=int, nargs="+", default=None,
        help="Specific sample indices to export. If not specified, exports all.",
    )
    parser.add_argument(
        "--sample", type=int, default=0,
        help="Inspect specific sample by index.",
    )
    parser.add_argument(
        "--trajectory", type=int, nargs=2, default=None, metavar=("START", "END"),
        help="Load trajectory between START and END sample indices.",
    )

    args = parser.parse_args()

    # Check if file exists
    if not os.path.exists(args.hdf5_path):
        print(f"Error: File not found: {args.hdf5_path}")
        sys.exit(1)

    # Load data
    loader = HDF5CameraDataLoader(args.hdf5_path)

    if args.summary:
        loader.print_summary()
        return

    if args.export_images:
        loader.export_images(
            output_dir=args.export_images,
            indices=args.export_indices,
        )
        return

    # Inspect single sample
    print(f"\nInspecting sample {args.sample}:")
    sample = loader.get_sample(args.sample)
    for key, value in sample.items():
        if isinstance(value, np.ndarray):
            print(f"  {key}: shape={value.shape}, dtype={value.dtype}")
        elif isinstance(value, bool):
            print(f"  {key}: {value}")
        elif isinstance(value, (int, float)):
            print(f"  {key}: {value:.6f}")
        else:
            print(f"  {key}: {type(value)}")

    # Load trajectory if requested
    if args.trajectory:
        start, end = args.trajectory
        trajectory = loader.get_trajectory(start, end)
        print(f"\nTrajectory shape: {len(trajectory['time/monotonic_s'])} samples")


if __name__ == "__main__":
    main()
