"""
replay_hdf5.py

Read a recorded teleoperation HDF5 file and replay the arm + hand movements
through RobotPoseController, preserving the original timing cadence.

The script performs two phases:
    1. Move to fixed teleop start joints [0, 25, 90, 0, 60, 0]
    2. Replay: execute the full recorded trajectory at the original speed

The script feeds arm/smoothed_pose (already safety-filtered during recording)
and hand/leap_pose into the controller, which still runs them through all the
same safety bounds before sending to hardware — so double protection is in place.

For smooth replay of pre-recorded trajectories, use:
  --trajectory-mode 2  (filter mode for maximum smoothing, vs. 1 for curve-fitting)
  --trajectory-radio 500-800  (smoothing coefficient; higher = smoother)

Usage
-----
    # Real-time replay with smooth filter mode (default)
    python replay_hdf5.py Combined/logs/teleop_20240101_120000.hdf5

    # Half-speed replay, no hand
    python replay_hdf5.py recording.hdf5 --speed 0.5 --no-hand

    # Maximum smoothing (mode 2, radio 800)
    python replay_hdf5.py recording.hdf5 --trajectory-mode 2 --trajectory-radio 800

    # Responsive curve-fitting mode (for comparison to live teleop)
    python replay_hdf5.py recording.hdf5 --trajectory-mode 1 --trajectory-radio 50

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


START_JOINT_DEG = [0.0, 25.0, 90.0, 0.0, 60.0, 0.0]


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


def _move_robot_to_start_joints(ctrl: RobotPoseController):
    """Move to the same deterministic start joint pose used by live teleoperation."""
    target_joints = list(START_JOINT_DEG)
    robot = ctrl.robot

    print(
        "\n--- Phase 0: Move to teleop start joints ---\n"
        "Target joints (deg): " + " ".join(f"{v:.1f}" for v in target_joints)
    )

    move_attempts = [
        ("rm_movej", (target_joints, 20, 0, 0, 1)),
        ("rm_movej", (target_joints, 20, 0, 1)),
        ("rm_movej", (target_joints, 20, 0, 0)),
        ("rm_movej", (target_joints, 20, 0)),
        ("rm_movej_p", (target_joints, 20, 0, 0, 1)),
        ("rm_movej_p", (target_joints, 20, 0, 1)),
    ]

    last_exc = None
    for method_name, args in move_attempts:
        method = getattr(robot, method_name, None)
        if method is None:
            continue
        try:
            ret = method(*args)
            if ret == 0:
                time.sleep(0.5)
                print("Reached teleop start joints.")
                return
            last_exc = RuntimeError(f"{method_name} returned {ret}")
        except TypeError:
            continue
        except Exception as exc:
            last_exc = exc

    raise RuntimeError(
        f"Failed to move to teleop start joints {target_joints}. Last error: {last_exc}"
    )


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
    trajectory_mode: int = 2,
    trajectory_radio: int = 500,
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
    print("\nConnecting to robot hardware...")
    ctrl = RobotPoseController(
        robot_ip=robot_ip,
        robot_port=robot_port,
        hand_port=hand_port,
        connect_hand=connect_hand,
    )
    print(f"Replay settings: trajectory_mode={trajectory_mode}, trajectory_radio={trajectory_radio}")

    # ------------------------------------------------------------------
    # Phase 0: Move to deterministic teleop start joints
    # ------------------------------------------------------------------
    _move_robot_to_start_joints(ctrl)

    # Seed safety filter with the first replay pose to avoid huge jump clamps
    first_valid_pose = None
    for i in range(n):
        if not np.any(np.isnan(arm_poses[i])):
            first_valid_pose = arm_poses[i].tolist()
            break

    if first_valid_pose:
        ctrl.safety.seed(first_valid_pose)
        print(f"Safety filter re-seeded with first replay pose: {[f'{v:.4f}' for v in first_valid_pose]}")

    # ------------------------------------------------------------------
    # Phase 1: Countdown before replay
    # ------------------------------------------------------------------
    print(f"\n--- Phase 1: Replay ({n} samples) ---")
    print(f"Starting replay in {start_delay:.0f} seconds — press Ctrl+C to abort.")
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
    try:
        for i in range(n):
            loop_t = time.perf_counter()

            arm_pose = arm_poses[i]
            hand_pose = hand_poses[i] if has_glove[i] else None

            # Skip frames with NaN arm data (can occur at logger startup)
            if np.any(np.isnan(arm_pose)):
                if i < 5:  # Only warn on first few
                    print(f"  (skipping sample {i}: arm pose is NaN)")
                continue

            # Sanity check: if first sample is all zeros, warn
            if i == 0 and np.allclose(arm_pose, 0):
                print(f"\n[WARNING] First arm pose is all zeros. Recording may be corrupted.")
                print(f"  Pose: {arm_pose.tolist()}")
                print(f"  Continuing anyway...\n")

            # --- Send arm with smooth trajectory parameters for replay ---
            ret = ctrl.send_arm_pose(arm_pose.tolist(), trajectory_mode, trajectory_radio)
            if ret != 0:
                print(f"\n[ERROR] CANFD status {ret} at sample {i}. Arm may not be connected.")
                print(f"  Pose sent: {arm_pose.tolist()}")
                print(f"  Trajectory mode: {trajectory_mode}, radio: {trajectory_radio}")
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
    p.add_argument(
        "--trajectory-mode", type=int, default=2,
        help="RealMan arm trajectory mode: 0=passthrough, 1=curve-fitting, 2=filter (best for replay).",
    )
    p.add_argument(
        "--trajectory-radio", type=int, default=500,
        help="Trajectory smoothing coefficient (mode 1: 0-100, mode 2: 0-999).",
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
        trajectory_mode=args.trajectory_mode,
        trajectory_radio=args.trajectory_radio,
    )


if __name__ == "__main__":
    main()
