"""
replay_hdf5.py

Read a recorded teleoperation HDF5 file and replay the arm + hand movements
through RobotPoseController, preserving the original timing cadence.

The script performs two phases:
    1. Move to fixed teleop start joints [0, 25, 90, 0, 60, 0]
    2. Replay: execute the full recorded trajectory at the original speed

Usage
-----
    # Real-time replay with smooth filter mode (default)
    python replay_hdf5.py src/logs/teleop_20240101_120000.hdf5

    # Replay AND show the recorded RealSense video feed in sync
    python replay_hdf5.py recording.hdf5 --show-video
"""

import argparse
import os
import sys
import time
import numpy as np

try:
    import h5py
except ImportError:
    print("ERROR: h5py is not installed. Run:  pip install h5py")
    sys.exit(1)

# Optional OpenCV for video playback
try:
    import cv2
except ImportError:
    cv2 = None

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from robot_pose_controller import RobotPoseController  # noqa: E402

START_JOINT_DEG = [0.0, 25.0, 90.0, 0.0, 60.0, 0.0]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _load_recording(hdf5_path: str):
    """Load all replay-relevant arrays from an HDF5 file."""
    f = h5py.File(hdf5_path, "r")
    
    arm_poses   = f["arm/smoothed_pose"][:]     # (N, 6)  float64
    hand_poses  = f["hand/leap_pose"][:]         # (N, 16) float64
    timestamps  = f["time/monotonic_s"][:]       # (N,)    float64

    # Backwards compatibility: Check for explicit glove boolean, otherwise infer from NaNs
    if "hand/has_glove_data" in f:
        has_glove = f["hand/has_glove_data"][:]
    else:
        has_glove = ~np.isnan(hand_poses[:, 0])

    meta = {k: f.attrs[k] for k in f.attrs}
    
    # Do not load camera data into RAM (it's too large). Just check if it exists.
    has_camera = "camera/color" in f

    return arm_poses, hand_poses, has_glove, timestamps, meta, has_camera, f


def _print_metadata(meta: dict, n_samples: int, timestamps: np.ndarray, has_camera: bool):
    print("\n--- Recording metadata ---")
    for k in ["created_utc", "robot_ip", "control_hz", "hand_side", "script", "zmq_endpoint"]:
        if k in meta:
            print(f"  {k:20s}: {meta[k]}")
    duration = float(timestamps[-1] - timestamps[0]) if len(timestamps) > 1 else 0.0
    print(f"  {'samples':20s}: {n_samples}")
    print(f"  {'duration_s':20s}: {duration:.3f}")
    print(f"  {'contains_video':20s}: {has_camera}")
    print("--------------------------\n")


def _move_robot_to_start_joints(ctrl: RobotPoseController):
    target_joints = list(START_JOINT_DEG)
    robot = ctrl.robot

    print("\n--- Phase 0: Move to teleop start joints ---")
    
    # 1. Get current joints BEFORE moving to bypass API caching bugs
    res, curr_joints = robot.rm_get_joint_degree()
    sleep_time = 3.0  # Minimum default wait
    
    if res == 0 and curr_joints:
        max_dist = max(abs(curr_joints[i] - target_joints[i]) for i in range(6))
        
        if max_dist < 1.0:
            print("Arm is already at start joints. Proceeding.")
            time.sleep(0.5)
            return
            
        # Time = Distance / Speed (20 deg/s) + 1.5s buffer for acceleration/deceleration
        sleep_time = (max_dist / 20.0) + 1.5
        print(f"Arm needs to move {max_dist:.1f} degrees.")
        print(f"Calculating travel time: {sleep_time:.1f} seconds...")
    else:
        print("[WARNING] Could not read current joints. Using conservative 10s travel wait.")
        sleep_time = 10.0

    print("Target joints (deg): " + " ".join(f"{v:.1f}" for v in target_joints))

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
                print(f"Command accepted. Blocking for {sleep_time:.1f}s while physical arm moves...")
                
                # --- The foolproof blocking wait ---
                time.sleep(sleep_time)
                
                print("Reached teleop start joints.")
                return
                
            last_exc = RuntimeError(f"{method_name} returned {ret}")
        except TypeError:
            continue
        except Exception as exc:
            last_exc = exc

    raise RuntimeError(f"Failed to move to teleop start joints {target_joints}. Last error: {last_exc}")
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
    trajectory_mode: int,
    trajectory_radio: int,
    show_video: bool,
):
    if not os.path.isfile(hdf5_path):
        print(f"ERROR: File not found: {hdf5_path}")
        sys.exit(1)

    print(f"Loading {hdf5_path} ...")
    arm_poses, hand_poses, has_glove, timestamps, meta, has_camera, h5_file = _load_recording(hdf5_path)
    n = arm_poses.shape[0]
    _print_metadata(meta, n, timestamps, has_camera)

    if show_video and not has_camera:
        print("[WARNING] --show-video requested, but this HDF5 file does not contain camera data.")
        show_video = False
    elif show_video and cv2 is None:
        print("[WARNING] --show-video requested, but opencv-python is not installed.")
        show_video = False

    # Dry-run
    if dry_run:
        print("[DRY RUN] No hardware connection made.\n")
        duration = float(timestamps[-1] - timestamps[0]) if n > 1 else 0.0
        print(f"  Samples            : {n}")
        print(f"  Recording duration : {duration:.3f} s  ({duration / speed:.3f} s at {speed}x)")
        glove_count = int(np.sum(has_glove))
        print(f"  Samples with glove : {glove_count} / {n}")
        h5_file.close()
        return

    # Connect hardware
    print("\nConnecting to robot hardware...")
    ctrl = RobotPoseController(
        robot_ip=robot_ip,
        robot_port=robot_port,
        hand_port=hand_port,
        connect_hand=connect_hand,
    )
    
    _move_robot_to_start_joints(ctrl)

    # Seed safety filter
    for i in range(n):
        if not np.any(np.isnan(arm_poses[i])):
            ctrl.safety.seed(arm_poses[i].tolist())
            break

    # Countdown
    print(f"\n--- Phase 1: Replay ({n} samples) ---")
    deadline = time.perf_counter() + start_delay
    while time.perf_counter() < deadline:
        print(f"\r  {deadline - time.perf_counter():.1f}s ... ", end="", flush=True)
        time.sleep(0.1)
    print("\r  Go!                ")

    # Replay loop
    replay_wall_start = time.perf_counter()
    try:
        for i in range(n):
            loop_t = time.perf_counter()

            arm_pose = arm_poses[i]
            hand_pose = hand_poses[i] if has_glove[i] else None

            if np.any(np.isnan(arm_pose)):
                continue

            # Command Arm
            ret = ctrl.send_arm_pose(arm_pose.tolist(), trajectory_mode, trajectory_radio)
            if ret != 0:
                print(f"\n[ERROR] CANFD status {ret} at sample {i}. Arm may not be connected.")
                break

            # Command Hand
            if hand_pose is not None and not np.any(np.isnan(hand_pose)):
                ctrl.send_hand_pose(hand_pose.tolist())

            # Optional: Display Video (skipping frames to maintain 125Hz performance)
            if show_video and i % 4 == 0: 
                frame = h5_file["camera/color"][i]
                cv2.imshow("Replay Camera (Recorded)", frame)
                cv2.waitKey(1)

            # Terminal output
            if i % 10 == 0:
                pct = (i + 1) / n * 100.0
                arm_str = " ".join(f"{v:7.4f}" for v in arm_pose)
                hand_str = "hand:on " if (hand_pose is not None) else "hand:off"
                print(f"\r[{pct:5.1f}%] {i + 1:6d}/{n}  arm: {arm_str}  {hand_str}  ", end="", flush=True)

            # Timing Cadence
            if i + 1 < n:
                target_t = loop_t + (float(timestamps[i + 1] - timestamps[i]) / max(speed, 1e-6))
                while time.perf_counter() < target_t:
                    pass

        elapsed = time.perf_counter() - replay_wall_start
        print(f"\nReplay complete: {n} samples in {elapsed:.2f} s.")

    except KeyboardInterrupt:
        print("\nReplay stopped by user.")

    finally:
        h5_file.close()
        if show_video:
            cv2.destroyAllWindows()
        ctrl.shutdown()

def _build_parser():
    p = argparse.ArgumentParser(description="Replay a teleoperation HDF5 recording on the real robot.")
    p.add_argument("hdf5_path", help="Path to the HDF5 file.")
    p.add_argument("--robot-ip", default="192.168.1.18")
    p.add_argument("--robot-port", type=int, default=8080)
    p.add_argument("--hand-port", default=None)
    p.add_argument("--speed", type=float, default=1.0)
    p.add_argument("--no-hand", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--start-delay", type=float, default=3.0)
    p.add_argument("--trajectory-mode", type=int, default=2)
    p.add_argument("--trajectory-radio", type=int, default=500)
    p.add_argument("--show-video", action="store_true", help="Playback the recorded camera view concurrently.")
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
        show_video=args.show_video,
    )

if __name__ == "__main__":
    main()