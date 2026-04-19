"""
replay_hdf5.py

Read a recorded teleoperation HDF5 file and replay the arm + hand movements
through RobotPoseController, preserving the original timing cadence.

The script feeds arm/smoothed_pose (already safety-filtered during recording)
and hand/leap_pose into the controller, which still runs them through all the
same safety bounds before sending to hardware — so double protection is in place.

Usage
-----
    # Real-time replay (default)
    python replay_hdf5.py Combined/logs/teleop_20240101_120000.hdf5

    # Half-speed replay, no hand
    python replay_hdf5.py recording.hdf5 --speed 0.5 --no-hand

    # Inspect a file without touching hardware
    python replay_hdf5.py recording.hdf5 --dry-run

    # Override robot IP
    python replay_hdf5.py recording.hdf5 --robot-ip 192.168.1.18
"""

import argparse
import os
import sys
import time

import numpy as np

# ---------------------------------------------------------------------------
# Make sure robot_pose_controller is importable when this script is run from
# outside the Combined/ directory.
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

try:
    import h5py
except ImportError:
    print("ERROR: h5py is not installed. Run:  pip install h5py")
    sys.exit(1)

from robot_pose_controller import RobotPoseController  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_recording(hdf5_path: str):
    """Load all replay-relevant arrays from an HDF5 file."""
    with h5py.File(hdf5_path, "r") as f:
        arm_poses   = f["arm/smoothed_pose"][:]     # (N, 6)  float64
        hand_poses  = f["hand/leap_pose"][:]         # (N, 16) float64
        has_glove   = f["hand/has_glove_data"][:]    # (N,)    bool
        timestamps  = f["time/monotonic_s"][:]       # (N,)    float64

        meta = {k: f.attrs[k] for k in f.attrs}

    return arm_poses, hand_poses, has_glove, timestamps, meta


def _print_metadata(meta: dict, n_samples: int, timestamps: np.ndarray):
    print("\n--- Recording metadata ---")
    for k in ["created_utc", "robot_ip", "control_hz", "hand_side", "script", "zmq_endpoint"]:
        if k in meta:
            print(f"  {k:20s}: {meta[k]}")
    duration = float(timestamps[-1] - timestamps[0]) if len(timestamps) > 1 else 0.0
    print(f"  {'samples':20s}: {n_samples}")
    print(f"  {'duration_s':20s}: {duration:.3f}")
    print("--------------------------\n")


# ---------------------------------------------------------------------------
# Core replay loop
# ---------------------------------------------------------------------------

def replay(
    hdf5_path: str,
    robot_ip: str,
    robot_port: int,
    hand_port: str,
    speed: float,
    connect_hand: bool,
    dry_run: bool,
    start_delay: float,
):
    if not os.path.isfile(hdf5_path):
        print(f"ERROR: File not found: {hdf5_path}")
        sys.exit(1)

    print(f"Loading {hdf5_path} ...")
    arm_poses, hand_poses, has_glove, timestamps, meta = _load_recording(hdf5_path)
    n = arm_poses.shape[0]
    _print_metadata(meta, n, timestamps)

    # ------------------------------------------------------------------
    # Dry-run: just inspect and exit
    # ------------------------------------------------------------------
    if dry_run:
        print("[DRY RUN] No hardware connection made.\n")
        duration = float(timestamps[-1] - timestamps[0]) if n > 1 else 0.0
        print(f"  Samples            : {n}")
        print(f"  Recording duration : {duration:.3f} s  ({duration / speed:.3f} s at {speed}x)")
        print(f"  First arm pose     : {arm_poses[0].tolist()}")
        print(f"  Last  arm pose     : {arm_poses[-1].tolist()}")
        glove_count = int(np.sum(has_glove))
        print(f"  Samples with glove : {glove_count} / {n}")
        return

    # ------------------------------------------------------------------
    # Connect to hardware
    # ------------------------------------------------------------------
    ctrl = RobotPoseController(
        robot_ip=robot_ip,
        robot_port=robot_port,
        hand_port=hand_port,
        connect_hand=connect_hand,
    )

    # ------------------------------------------------------------------
    # Countdown before movement
    # ------------------------------------------------------------------
    print(f"\nStarting replay in {start_delay:.0f} seconds — press Ctrl+C to abort.")
    deadline = time.perf_counter() + start_delay
    while time.perf_counter() < deadline:
        remaining = deadline - time.perf_counter()
        print(f"\r  {remaining:.1f}s ... ", end="", flush=True)
        time.sleep(0.1)
    print("\r  Go!                ")

    # ------------------------------------------------------------------
    # Replay loop
    # ------------------------------------------------------------------
    replay_wall_start = time.perf_counter()
    rec_t0 = float(timestamps[0])

    try:
        for i in range(n):
            loop_t = time.perf_counter()

            arm_pose = arm_poses[i]
            hand_pose = hand_poses[i] if has_glove[i] else None

            # Skip frames with NaN arm data (can occur at logger startup)
            if np.any(np.isnan(arm_pose)):
                continue

            # --- Send arm ---
            ret = ctrl.send_arm_pose(arm_pose.tolist())
            if ret != 0:
                print(f"\nCANFD error at sample {i}: {ret} — stopping replay.")
                break

            # --- Send hand (only when glove data was active during recording) ---
            if hand_pose is not None and not np.any(np.isnan(hand_pose)):
                ctrl.send_hand_pose(hand_pose.tolist())

            # --- Progress display ---
            pct = (i + 1) / n * 100.0
            arm_str = " ".join(f"{v:7.4f}" for v in arm_pose)
            hand_str = "hand:on " if (hand_pose is not None) else "hand:off"
            print(
                f"\r[{pct:5.1f}%] {i + 1:6d}/{n}  arm: {arm_str}  {hand_str}  ",
                end="",
                flush=True,
            )

            # --- Timing: maintain original inter-frame cadence scaled by speed ---
            if i + 1 < n:
                original_dt = float(timestamps[i + 1] - timestamps[i])
                target_dt = original_dt / max(speed, 1e-6)
                target_t = loop_t + target_dt
                # Precision spin-wait (same approach as the teleop scripts)
                while time.perf_counter() < target_t:
                    pass

        elapsed = time.perf_counter() - replay_wall_start
        print(f"\nReplay complete: {n} samples in {elapsed:.2f} s.")

    except KeyboardInterrupt:
        print("\nReplay stopped by user.")

    finally:
        ctrl.shutdown()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _build_parser():
    p = argparse.ArgumentParser(
        description="Replay a teleoperation HDF5 recording on the real robot.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "hdf5_path",
        help="Path to the HDF5 file produced by combined_simple_teleop_real_logger.py.",
    )
    p.add_argument("--robot-ip", default="192.168.1.18", help="RealMan arm IP address.")
    p.add_argument("--robot-port", type=int, default=8080, help="RealMan arm TCP port.")
    p.add_argument("--hand-port", default=None, help="Serial port for LEAP hand (auto-detected if omitted).")
    p.add_argument(
        "--speed", type=float, default=1.0,
        help="Playback speed multiplier. 1.0 = real-time, 0.5 = half-speed, 2.0 = double-speed.",
    )
    p.add_argument("--no-hand", action="store_true", help="Skip LEAP hand replay.")
    p.add_argument(
        "--dry-run", action="store_true",
        help="Inspect the HDF5 file and print a summary without connecting to hardware.",
    )
    p.add_argument(
        "--start-delay", type=float, default=3.0,
        help="Seconds to wait (with countdown) before motion begins.",
    )
    return p


def main():
    args = _build_parser().parse_args()
    replay(
        hdf5_path=args.hdf5_path,
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        hand_port=args.hand_port,
        speed=args.speed,
        connect_hand=not args.no_hand,
        dry_run=args.dry_run,
        start_delay=args.start_delay,
    )


if __name__ == "__main__":
    main()
