import argparse
from datetime import datetime
import importlib
import os
import sys
import threading
import time

import numpy as np
import zmq
from scipy.spatial.transform import Rotation as R, Slerp

try:
    import h5py
except ImportError:
    h5py = None

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
COMBINED_DIR = os.path.dirname(__file__)
REALMAN_DIR = os.path.join(REPO_ROOT, "RMAPI", "Python")
MANUS_PY_DIR = os.path.join(REPO_ROOT, "LMAPI", "python")

for path in [COMBINED_DIR, REALMAN_DIR, MANUS_PY_DIR]:
    if path not in sys.path:
        sys.path.insert(0, path)

_teleop_import_error = None
try:
    from teleoperate import ViveToRMMapper  # noqa: E402
except Exception as exc:
    _teleop_import_error = exc
    ViveToRMMapper = None

try:
    DynamixelClient = importlib.import_module(
        "leap_hand_utils.dynamixel_client"
    ).DynamixelClient
    lhu = importlib.import_module("leap_hand_utils.leap_hand_utils")
except Exception:
    DynamixelClient = None
    lhu = None


class RealSenseD435iCapture:
    """
    Captures RGB and depth frames from Intel RealSense D435i in a background thread.
    Applies frame synchronization and stores latest frame with timestamp.
    """

    def __init__(
        self,
        width=640,
        height=480,
        fps=30,
        enable_rgb=True,
        enable_depth=True,
    ):
        if rs is None:
            raise RuntimeError(
                "RealSense support requested but pyrealsense2 not installed. "
                "Install with: pip install pyrealsense2"
            )

        self.width = width
        self.height = height
        self.fps = fps
        self.enable_rgb = enable_rgb
        self.enable_depth = enable_depth

        self._latest_frame = None
        self._latest_timestamp = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()

        # Setup pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure streams
        if self.enable_rgb:
            self.config.enable_stream(
                rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps
            )
        if self.enable_depth:
            self.config.enable_stream(
                rs.stream.depth, self.width, self.height, rs.format.z16, self.fps
            )

        # Start pipeline
        self.profile = self.pipeline.start(self.config)

        # Get camera intrinsics for potential later use
        self.color_intrinsics = None
        self.depth_intrinsics = None

        if self.enable_rgb:
            color_profile = self.profile.get_stream(rs.stream.color)
            self.color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()

        if self.enable_depth:
            depth_profile = self.profile.get_stream(rs.stream.depth)
            self.depth_intrinsics = depth_profile.as_video_stream_profile().get_intrinsics()

        # Align depth frames to RGB for consistency
        self.align = rs.align(rs.stream.color)

        # Start capture thread
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        """Background thread for continuous frame capture."""
        while not self._stop_event.is_set():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)

                # Align depth to color frame
                aligned_frames = self.align.process(frames)

                frame_data = {}

                if self.enable_rgb:
                    color_frame = aligned_frames.get_color_frame()
                    if color_frame:
                        frame_data["rgb"] = np.asanyarray(color_frame.get_data())

                if self.enable_depth:
                    depth_frame = aligned_frames.get_depth_frame()
                    if depth_frame:
                        depth_data = np.asanyarray(depth_frame.get_data())
                        frame_data["depth"] = depth_data

                # Get timestamp from frame
                if frames:
                    timestamp = frames.get_timestamp()
                    frame_data["timestamp"] = timestamp / 1000.0  # Convert ms to seconds

                # Update with lock
                with self._frame_lock:
                    self._latest_frame = frame_data
                    self._latest_timestamp = time.perf_counter()

            except Exception as e:
                print(f"[RealSense] Capture error: {e}")
                time.sleep(0.01)

    @property
    def latest_frame(self):
        """Get latest captured frame data safely."""
        with self._frame_lock:
            return self._latest_frame.copy() if self._latest_frame else None

    @property
    def latest_timestamp(self):
        """Get timestamp of latest frame capture."""
        with self._frame_lock:
            return self._latest_timestamp

    def close(self):
        """Stop capture and close pipeline."""
        self._stop_event.set()
        try:
            self._thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            self.pipeline.stop()
        except Exception:
            pass


class TeleopHDF5Logger:
    """
    Streams teleoperation samples into an HDF5 file using resizable datasets.
    """

    def __init__(self, output_path, args):
        if h5py is None:
            raise RuntimeError(
                "HDF5 logging requested, but h5py is not installed. "
                "Install it with: pip install h5py"
            )

        self.output_path = output_path
        self.sample_count = 0
        self.flush_every = max(1, int(args.log_flush_every))

        dir_path = os.path.dirname(self.output_path)
        if dir_path:
            os.makedirs(dir_path, exist_ok=True)
        self.file = h5py.File(self.output_path, "w")

        self.file.attrs["created_utc"] = datetime.utcnow().isoformat() + "Z"
        self.file.attrs["robot_ip"] = args.robot_ip
        self.file.attrs["robot_port"] = args.robot_port
        self.file.attrs["zmq_endpoint"] = args.zmq_endpoint
        self.file.attrs["hand_side"] = args.hand_side
        self.file.attrs["control_hz"] = args.control_hz
        self.file.attrs["arm_pos_scale"] = args.arm_pos_scale
        self.file.attrs["arm_rot_scale"] = args.arm_rot_scale
        self.file.attrs["calibration_countdown"] = args.calibration_countdown
        self.file.attrs["hand_current_limit"] = args.hand_current_limit
        self.file.attrs["script"] = "combined_simple_teleop_real_logger.py"

        # Camera configuration
        self.has_camera = args.enable_realsense
        self.camera_width = args.realsense_width
        self.camera_height = args.realsense_height
        self.camera_fps = args.realsense_fps

        self.datasets = {
            "time/monotonic_s": self._create_dataset("time/monotonic_s", (0,), (None,), np.float64),
            "time/wall_time_s": self._create_dataset("time/wall_time_s", (0,), (None,), np.float64),
            "arm/raw_pose": self._create_dataset("arm/raw_pose", (0, 6), (None, 6), np.float64),
            "arm/bounded_pose": self._create_dataset("arm/bounded_pose", (0, 6), (None, 6), np.float64),
            "arm/safe_pose": self._create_dataset("arm/safe_pose", (0, 6), (None, 6), np.float64),
            "arm/smoothed_pose": self._create_dataset("arm/smoothed_pose", (0, 6), (None, 6), np.float64),
            "arm/hold_flag": self._create_dataset("arm/hold_flag", (0,), (None,), np.bool_),
            "arm/canfd_status": self._create_dataset("arm/canfd_status", (0,), (None,), np.int32),
            "hand/manus_joints": self._create_dataset("hand/manus_joints", (0, 20), (None, 20), np.float64),
            "hand/leap_pose": self._create_dataset("hand/leap_pose", (0, 16), (None, 16), np.float64),
            "hand/has_glove_data": self._create_dataset("hand/has_glove_data", (0,), (None,), np.bool_),
        }

        # Camera datasets - created only if camera is enabled
        if self.has_camera:
            # Store RGB frames as variable-length uint8 arrays (flattened for HDF5 storage)
            vlen_uint8 = h5py.special_dtype(vlen=np.uint8)
            self.datasets["camera/rgb"] = self._create_dataset(
                "camera/rgb",
                (0,),
                (None,),
                vlen_uint8,
            )
            # Store depth frames as variable-length uint16 arrays (flattened for HDF5 storage)
            vlen_uint16 = h5py.special_dtype(vlen=np.uint16)
            self.datasets["camera/depth"] = self._create_dataset(
                "camera/depth",
                (0,),
                (None,),
                vlen_uint16,
            )
            # Store camera frame timestamps for frame-accurate synchronization
            self.datasets["camera/frame_time"] = self._create_dataset(
                "camera/frame_time",
                (0,),
                (None,),
                np.float64,
            )
            # Store camera frame indices to track drops
            self.datasets["camera/frame_index"] = self._create_dataset(
                "camera/frame_index",
                (0,),
                (None,),
                np.int64,
            )
            # Store camera file attributes for reconstruction
            self.file.attrs["camera_width"] = self.camera_width
            self.file.attrs["camera_height"] = self.camera_height
            self.file.attrs["camera_fps"] = self.camera_fps
            self.file.attrs["camera_enabled"] = True
        else:
            self.file.attrs["camera_enabled"] = False

    def _create_dataset(self, name, shape, maxshape, dtype):
        return self.file.create_dataset(
            name=name,
            shape=shape,
            maxshape=maxshape,
            dtype=dtype,
            chunks=True,
        )

    def append_sample(
        self,
        monotonic_s,
        wall_time_s,
        raw_pose,
        bounded_pose,
        safe_pose,
        smoothed_pose,
        hold_flag,
        canfd_status,
        glove_message,
        leap_pose,
        camera_frame=None,
        camera_frame_time=None,
        camera_frame_index=None,
    ):
        index = self.sample_count

        values = {
            "time/monotonic_s": float(monotonic_s),
            "time/wall_time_s": float(wall_time_s),
            "arm/raw_pose": np.asarray(
                raw_pose if raw_pose is not None else [np.nan] * 6, dtype=np.float64
            ),
            "arm/bounded_pose": np.asarray(
                bounded_pose if bounded_pose is not None else [np.nan] * 6, dtype=np.float64
            ),
            "arm/safe_pose": np.asarray(
                safe_pose if safe_pose is not None else [np.nan] * 6, dtype=np.float64
            ),
            "arm/smoothed_pose": np.asarray(
                smoothed_pose if smoothed_pose is not None else [np.nan] * 6, dtype=np.float64
            ),
            "arm/hold_flag": bool(hold_flag),
            "arm/canfd_status": int(canfd_status),
            "hand/manus_joints": np.asarray(
                glove_message if glove_message is not None else [np.nan] * 20,
                dtype=np.float64,
            ),
            "hand/leap_pose": np.asarray(
                leap_pose if leap_pose is not None else [np.nan] * 16,
                dtype=np.float64,
            ),
            "hand/has_glove_data": bool(glove_message is not None),
        }

        # Add camera data if enabled
        if self.has_camera and camera_frame is not None:
            rgb_frame = camera_frame.get("rgb")
            depth_frame = camera_frame.get("depth")
            frame_timestamp = camera_frame.get("timestamp", 0.0)

            # Store RGB frame as flattened uint8 array
            if rgb_frame is not None:
                rgb_flat = np.asarray(rgb_frame, dtype=np.uint8).flatten()
                values["camera/rgb"] = rgb_flat
            else:
                values["camera/rgb"] = np.array([], dtype=np.uint8)

            # Store depth frame as flattened uint16 array
            if depth_frame is not None:
                depth_flat = np.asarray(depth_frame, dtype=np.uint16).flatten()
                values["camera/depth"] = depth_flat
            else:
                values["camera/depth"] = np.array([], dtype=np.uint16)

            # Store frame metadata
            values["camera/frame_time"] = float(camera_frame_time or frame_timestamp)
            values["camera/frame_index"] = int(camera_frame_index or -1)

        for name, value in values.items():
            dataset = self.datasets[name]
            new_shape = list(dataset.shape)
            new_shape[0] = index + 1
            dataset.resize(tuple(new_shape))
            dataset[index] = value

        self.sample_count += 1

        if self.sample_count % self.flush_every == 0:
            self.file.flush()

    def close(self):
        if getattr(self, "file", None) is not None:
            self.file.attrs["sample_count"] = self.sample_count
            self.file.flush()
            self.file.close()
            self.file = None


class ManusErgonomicsSubscriber:
    """
    Reads the 40-value MANUS ergonomics stream in a background thread and
    exposes the latest 20-value hand slice.
    """

    def __init__(self, endpoint="tcp://localhost:8000", hand_side="right"):
        self.endpoint = endpoint
        self.hand_side = hand_side
        self._message = None
        self._stop_event = threading.Event()

        context = zmq.Context()
        self.socket = context.socket(zmq.PULL)
        self.socket.connect(self.endpoint)

        self._thread = threading.Thread(target=self._update_value, daemon=True)
        self._thread.start()

    @property
    def message(self):
        return self._message

    def _update_value(self):
        while not self._stop_event.is_set():
            try:
                message = self.socket.recv(flags=zmq.NOBLOCK)
                data = message.decode("utf-8").split(",")

                if len(data) == 40:
                    if self.hand_side == "left":
                        self._message = list(map(float, data[0:20]))
                    else:
                        self._message = list(map(float, data[20:40]))
            except zmq.Again:
                time.sleep(0.001)

    def close(self):
        self._stop_event.set()
        try:
            self.socket.close(linger=0)
        except Exception:
            pass


class LeapHandDirectController:
    """
    Minimal LEAP Hand controller copied in spirit from the MANUS example:
    direct joint-angle mapping, no retargeting or IK.
    """

    def __init__(self, hand_port=None, current_limit=350):
        if DynamixelClient is None or lhu is None:
            raise RuntimeError(
                "leap_hand_utils not installed. Cannot run live LEAP Hand control."
            )
        self.kP = 400
        self.kI = 0
        self.kD = 300
        self.curr_lim = current_limit
        self.motors = list(range(16))
        self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
        self.last_command = np.array(self.curr_pos, dtype=np.float64)

        port_candidates = []
        if hand_port:
            port_candidates.append(hand_port)
        port_candidates.extend(["/dev/ttyUSB0", "/dev/ttyUSB1", "COM13"])

        last_error = None
        self.dxl_client = None
        for candidate in port_candidates:
            try:
                self.dxl_client = DynamixelClient(self.motors, candidate, 4000000)
                self.dxl_client.connect()
                self.port = candidate
                break
            except Exception as exc:
                last_error = exc
                self.dxl_client = None

        if self.dxl_client is None:
            raise RuntimeError(f"Failed to connect to LEAP Hand: {last_error}")

        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)
        self.dxl_client.sync_write(
            self.motors, np.ones(len(self.motors)) * self.kP, 84, 2
        )
        self.dxl_client.sync_write([0, 4, 8], np.ones(3) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(
            self.motors, np.ones(len(self.motors)) * self.kI, 82, 2
        )
        self.dxl_client.sync_write(
            self.motors, np.ones(len(self.motors)) * self.kD, 80, 2
        )
        self.dxl_client.sync_write([0, 4, 8], np.ones(3) * (self.kD * 0.75), 80, 2)
        self.dxl_client.sync_write(
            self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2
        )
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def convert_manus_to_leap_pose(self, hand_joints):
        """
        Direct-copy mapping from MANUS ergonomics data to LEAP-Hand-compatible
        Allegro-style joint ordering. This follows the simple demo approach.
        """

        if len(hand_joints) != 20:
            raise ValueError(f"Expected 20 MANUS joint values, got {len(hand_joints)}")

        pose = np.deg2rad(
            hand_joints[4:8]
            + [hand_joints[8] + 10]
            + hand_joints[9:16]
            + [90 - 1.75 * hand_joints[1]]
            + [-45 + 3.0 * hand_joints[0]]
            + [-30 + 3.0 * hand_joints[2]]
            + [hand_joints[3]]
        )
        pose[0] = -2.5 * pose[0] + np.deg2rad(20)
        pose[1] = 1.5 * pose[1]
        pose[4] = -2.5 * pose[4] + np.deg2rad(30)
        pose[5] = 1.5 * pose[5]
        pose[8] = -2.5 * pose[8]
        pose[9] = 1.5 * pose[9]
        pose[12] = 1.5 * pose[12]
        pose[13] = 1.5 * pose[13] + np.deg2rad(90)
        return lhu.allegro_to_LEAPhand(pose, zeros=False)

    def send_manus_command(self, hand_joints):
        leap_pose = self.convert_manus_to_leap_pose(hand_joints)
        self.curr_pos = np.array(leap_pose)
        self.last_command = self.curr_pos.copy()
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
        return self.last_command


class HighFrequencyInterpolator:
    """
    Exponential Moving Average (EMA) to upsample the lower-frequency 
    VR tracker data into a smooth, high-frequency stream for the robot.
    Optimized strictly for 6D poses [X, Y, Z, Rx, Ry, Rz] with position EMA
    and quaternion SLERP for rotation.
    """
    def __init__(self, alpha_pos=0.15, alpha_rot=0.15):
        self.current_pose = None
        self.alpha_pos = alpha_pos
        self.alpha_rot = alpha_rot

    def step(self, target_pose):
        # Convert target to a flat numpy array to avoid list reference bugs
        target = np.array(target_pose, dtype=np.float64)

        if self.current_pose is None:
            self.current_pose = target.copy()
            return self.current_pose.tolist()

        # 1. Position Interpolation (X, Y, Z) - Standard Linear EMA
        self.current_pose[:3] += self.alpha_pos * (target[:3] - self.current_pose[:3])

        # 2. Rotation Interpolation (Rx, Ry, Rz) - Quaternion SLERP
        t = float(np.clip(self.alpha_rot, 0.0, 1.0))
        if t <= 0.0:
            return self.current_pose.tolist()
        if t >= 1.0:
            self.current_pose[3:] = target[3:]
            return self.current_pose.tolist()

        rot_curr = R.from_euler("xyz", self.current_pose[3:], degrees=False)
        rot_targ = R.from_euler("xyz", target[3:], degrees=False)
        slerp = Slerp([0.0, 1.0], R.from_quat([rot_curr.as_quat(), rot_targ.as_quat()]))
        self.current_pose[3:] = slerp(t).as_euler("xyz", degrees=False)

        return self.current_pose.tolist()


class CombinedSimpleTeleop:
    START_JOINT_DEG = [0.0, 25.0, 90.0, 0.0, 60.0, 0.0]

    def __init__(self, args):
        self.args = args
        self.mapper = None
        self.hand = None
        self.glove = None
        self.camera = None
        self.logger = None
        self.interpolator = HighFrequencyInterpolator()
        self._last_cmd_pose = None
        self._last_recovery_t = 0.0
        self._camera_frame_index = 0

    @staticmethod
    def _wrap_angle(angle):
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    @classmethod
    def _shortest_angle_diff(cls, target, current):
        return cls._wrap_angle(target - current)

    def _stabilize_command_pose(self, pose_6d):
        """
        Keep rotational commands continuous across +/-pi and limit per-cycle
        angular step to avoid triggering onboard stop/freeze behavior.
        """
        cmd = np.asarray(pose_6d, dtype=np.float64).copy()
        if self._last_cmd_pose is None:
            self._last_cmd_pose = cmd.tolist()
            return cmd.tolist()

        last_cmd = np.asarray(self._last_cmd_pose, dtype=np.float64)
        max_rot_step = np.deg2rad(7.5)

        for idx in range(3, 6):
            last_wrapped = self._wrap_angle(last_cmd[idx])
            diff = self._shortest_angle_diff(cmd[idx], last_wrapped)
            diff = float(np.clip(diff, -max_rot_step, max_rot_step))
            cmd[idx] = last_cmd[idx] + diff

        self._last_cmd_pose = cmd.tolist()
        return cmd.tolist()

    def _recover_arm_stream_state(self):
        """Recover command stream state from current robot pose without stopping teleop."""
        now = time.perf_counter()
        if now - self._last_recovery_t < 0.5:
            return
        self._last_recovery_t = now

        try:
            self.mapper.robot.rm_set_arm_run_mode(1)
        except Exception:
            pass

        current_pose = self.mapper.get_current_robot_pose()
        if current_pose is None:
            return

        self.mapper.last_filtered_pose = list(current_pose)
        self.interpolator.current_pose = np.asarray(current_pose, dtype=np.float64)
        self._last_cmd_pose = list(current_pose)

    @staticmethod
    def _pose_snapshot(pose):
        if pose is None:
            return None
        return np.asarray(pose, dtype=np.float64).copy().tolist()

    def _compute_raw_pose(self):
        current_T = self.mapper.get_current_tracker_matrix()
        T_delta = np.linalg.inv(self.mapper.tracker_home_T) @ current_T
        pos_delta = T_delta[:3, 3]

        remapped_pos = np.array(
            [-pos_delta[1], -pos_delta[0], -pos_delta[2]], dtype=np.float64
        ) * self.mapper.pos_scale

        rotvec_delta = R.from_matrix(T_delta[:3, :3]).as_rotvec()
        remapped_rotvec = np.array(
            [-rotvec_delta[1], -rotvec_delta[0], -rotvec_delta[2]], dtype=np.float64
        ) * self.mapper.rot_scale
        euler_delta = R.from_rotvec(remapped_rotvec).as_euler("xyz", degrees=False)

        target_pose = np.asarray(self.mapper.robot_home_pose, dtype=np.float64).copy()
        target_pose[:3] += remapped_pos
        target_pose[3:] += euler_delta
        return target_pose.tolist()

    def _move_robot_to_start_joints(self):
        """Move to a fixed, known-safe start joint pose before teleop calibration."""
        target_joints = list(self.START_JOINT_DEG)
        robot = self.mapper.robot

        print(
            "\nMoving arm to start joints (deg): "
            + " ".join(f"{v:.1f}" for v in target_joints)
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
                    return
                last_exc = RuntimeError(f"{method_name} returned {ret}")
            except TypeError:
                continue
            except Exception as exc:
                last_exc = exc

        raise RuntimeError(
            f"Failed to move to start joints {target_joints}. Last error: {last_exc}"
        )

    def setup(self):
        if ViveToRMMapper is None:
            raise RuntimeError(
                "teleoperate import failed; live teleop unavailable. "
                f"Root cause: {_teleop_import_error}"
            )
        self.mapper = ViveToRMMapper(
            robot_ip=self.args.robot_ip,
            robot_port=self.args.robot_port,
        )
        self.mapper.pos_scale = self.args.arm_pos_scale
        self.mapper.rot_scale = self.args.arm_rot_scale

        # Force a deterministic start posture before calibration/teleop.
        self._move_robot_to_start_joints()

        self.hand = LeapHandDirectController(
            hand_port=self.args.hand_port,
            current_limit=self.args.hand_current_limit,
        )
        self.glove = ManusErgonomicsSubscriber(
            endpoint=self.args.zmq_endpoint,
            hand_side=self.args.hand_side,
        )
        if self.args.log_hdf5:
            log_path = self.args.log_path or self._default_log_path()
            self.logger = TeleopHDF5Logger(log_path, self.args)
            print(f"HDF5 logging enabled: {log_path}")

        # Initialize RealSense camera if enabled
        if self.args.enable_realsense:
            try:
                self.camera = RealSenseD435iCapture(
                    width=self.args.realsense_width,
                    height=self.args.realsense_height,
                    fps=self.args.realsense_fps,
                    enable_rgb=self.args.realsense_rgb,
                    enable_depth=self.args.realsense_depth,
                )
                print(
                    f"RealSense D435i initialized: {self.args.realsense_width}x{self.args.realsense_height} @ {self.args.realsense_fps} Hz"
                )
            except Exception as e:
                print(f"[WARNING] Failed to initialize RealSense camera: {e}")
                self.camera = None

        initial_pose = self.mapper.get_current_robot_pose()
        if initial_pose:
            time.sleep(1)

        self.mapper.calibrate(countdown=self.args.calibration_countdown)
        self._last_cmd_pose = list(self.mapper.robot_home_pose)

        if self.logger is not None:
            self.logger.file.attrs["tracker_key"] = getattr(self.mapper, "tracker_key", "")
            self.logger.file.attrs["robot_home_pose"] = np.asarray(
                self.mapper.robot_home_pose, dtype=np.float64
            )
            self.logger.file.attrs["tracker_home_T"] = np.asarray(
                self.mapper.tracker_home_T, dtype=np.float64
            )

    def _default_log_path(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(REPO_ROOT, "Combined", "logs", f"teleop_{timestamp}.hdf5")

    def run(self):
        self.setup()
        print(
            f"\nCombined teleop running at {self.args.control_hz:.1f} Hz. "
            "Press Ctrl+C to stop."
        )

        dt = 1.0 / self.args.control_hz
        warned_no_glove = False
        canfd_error_streak = 0

        while True:
            loop_start = time.perf_counter()
            wall_time_s = time.time()

            raw_pose = None
            bounded_pose = None
            try:
                raw_pose = self._compute_raw_pose()
                # teleoperate.py is used here as a safety utility only.
                bounded_pose = self.mapper.apply_safety_bounds(raw_pose)
            except Exception as exc:
                print(f"\n[WARNING] Falling back to hold pose: {exc}")

            if bounded_pose is not None:
                safe_pose = bounded_pose
                is_holding = False
            else:
                safe_pose = self.mapper.last_filtered_pose or self.mapper.robot_home_pose
                is_holding = True

            smoothed_pose = self.interpolator.step(safe_pose)
            smoothed_pose = self._stabilize_command_pose(smoothed_pose)

            raw_pose_log = self._pose_snapshot(raw_pose)
            bounded_pose_log = self._pose_snapshot(bounded_pose)
            safe_pose_log = self._pose_snapshot(safe_pose)
            smoothed_pose_log = self._pose_snapshot(smoothed_pose)

            # trajectory_mode=1 (curve fitting), radio=20 (reduced buffer)
            arm_ret = self.mapper.robot.rm_movep_canfd(smoothed_pose_log, True, 1, 20)
            arm_ok = (arm_ret == 0)
            if arm_ok:
                canfd_error_streak = 0
            else:
                canfd_error_streak += 1
                # Do not hard-stop teleop on transient onboard safety events.
                if canfd_error_streak == 1 or (canfd_error_streak % 20 == 0):
                    print(
                        f"\n[WARNING] CANFD transmission error: {arm_ret} "
                        f"(streak={canfd_error_streak})"
                    )
                # Periodically re-assert teleop run mode to recover from temporary arm state changes.
                if canfd_error_streak % 10 == 0:
                    try:
                        self.mapper.robot.rm_set_arm_run_mode(1)
                    except Exception:
                        pass
                # Hard recovery when error streak persists: re-seed command state from live arm pose.
                if canfd_error_streak % 25 == 0:
                    self._recover_arm_stream_state()

            glove_message = self.glove.message
            leap_pose = self.hand.last_command
            if glove_message is not None:
                warned_no_glove = False
                leap_pose = self.hand.send_manus_command(glove_message)
            elif not warned_no_glove:
                print("\nWaiting for MANUS ergonomics data on ZMQ...")
                warned_no_glove = True

            # Capture camera frame if available
            camera_frame = None
            camera_frame_time = None
            camera_frame_index = None
            if self.camera is not None:
                camera_frame = self.camera.latest_frame
                if camera_frame is not None:
                    camera_frame_time = camera_frame.get("timestamp", wall_time_s)
                    camera_frame_index = self._camera_frame_index
                    self._camera_frame_index += 1

            if self.logger is not None:
                self.logger.append_sample(
                    monotonic_s=loop_start,
                    wall_time_s=wall_time_s,
                    raw_pose=raw_pose_log,
                    bounded_pose=bounded_pose_log,
                    safe_pose=safe_pose_log,
                    smoothed_pose=smoothed_pose_log,
                    hold_flag=is_holding,
                    canfd_status=arm_ret,
                    glove_message=glove_message,
                    leap_pose=leap_pose,
                    camera_frame=camera_frame,
                    camera_frame_time=camera_frame_time,
                    camera_frame_index=camera_frame_index,
                )

            if not arm_ok:
                print(
                    "\r[WARNING] Arm command retrying after CANFD error. Hand still streaming.   ",
                    end="",
                    flush=True,
                )
            elif is_holding:
                print(
                    "\r[WARNING] Arm boundary limit active. Hand still streaming.   ",
                    end="",
                    flush=True,
                )
            else:
                formatted_pose = " ".join(f"{val:.4f}" for val in smoothed_pose_log)
                glove_state = "hand:on" if glove_message is not None else "hand:wait"
                print(
                    f"\rArm: {formatted_pose}   {glove_state}   ",
                    end="",
                    flush=True,
                )

            # Precision Spin-Wait Loop to guarantee sub-millisecond execution timing
            target_time = loop_start + dt
            while time.perf_counter() < target_time:
                pass
    def shutdown(self):
        if self.logger is not None:
            self.logger.close()
        if self.camera is not None:
            self.camera.close()
        if self.glove is not None:
            self.glove.close()
        if self.mapper is not None:
            try:
                self.mapper.robot.rm_delete_robot_arm()
            except Exception:
                pass
        print("\nDisconnected.")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Simple combined teleop: Vive tracker arm + MANUS direct-copy LEAP Hand + RealSense D435i camera"
    )
    parser.add_argument("--robot-ip", default="192.168.1.18")
    parser.add_argument("--robot-port", type=int, default=8080)
    parser.add_argument("--zmq-endpoint", default="tcp://localhost:8000")
    parser.add_argument("--hand-side", choices=["left", "right"], default="right")
    parser.add_argument("--hand-port", default=None)
    # Kept at 125.0 Hz to meet CANFD < 10ms cycle requirement
    parser.add_argument("--control-hz", type=float, default=125.0)
    parser.add_argument("--calibration-countdown", type=int, default=3)
    parser.add_argument("--arm-pos-scale", type=float, default=1.0)
    parser.add_argument("--arm-rot-scale", type=float, default=1.0)
    parser.add_argument("--hand-current-limit", type=int, default=350)
    parser.add_argument(
        "--log-hdf5", action="store_true", default=True,
        help="Enable HDF5 logging (on by default). Pass --no-log-hdf5 to disable.",
    )
    parser.add_argument(
        "--no-log-hdf5", dest="log_hdf5", action="store_false",
        help="Disable HDF5 logging.",
    )
    parser.add_argument("--log-path", default=None)
    parser.add_argument("--log-flush-every", type=int, default=50)

    # RealSense camera arguments
    parser.add_argument(
        "--enable-realsense", action="store_true", default=False,
        help="Enable Intel RealSense D435i depth camera integration.",
    )
    parser.add_argument(
        "--realsense-width", type=int, default=640,
        help="RealSense camera frame width in pixels.",
    )
    parser.add_argument(
        "--realsense-height", type=int, default=480,
        help="RealSense camera frame height in pixels.",
    )
    parser.add_argument(
        "--realsense-fps", type=int, default=30,
        help="RealSense camera FPS (frames per second).",
    )
    parser.add_argument(
        "--realsense-rgb", action="store_true", default=True,
        help="Capture RGB frames from RealSense (on by default).",
    )
    parser.add_argument(
        "--no-realsense-rgb", dest="realsense_rgb", action="store_false",
        help="Disable RGB frame capture.",
    )
    parser.add_argument(
        "--realsense-depth", action="store_true", default=True,
        help="Capture depth frames from RealSense (on by default).",
    )
    parser.add_argument(
        "--no-realsense-depth", dest="realsense_depth", action="store_false",
        help="Disable depth frame capture.",
    )

    return parser


def main():
    args = build_parser().parse_args()
    teleop = CombinedSimpleTeleop(args)
    try:
        teleop.run()
    except KeyboardInterrupt:
        print("\nTeleoperation stopped by user.")
    except Exception as exc:
        print(f"\nError: {exc}")
    finally:
        teleop.shutdown()


if __name__ == "__main__":
    main()