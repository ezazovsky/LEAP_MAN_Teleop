import argparse
from datetime import datetime
import os
import sys
import threading
import time

import numpy as np
import zmq

try:
    import h5py
except ImportError:
    h5py = None

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REALMAN_DIR = os.path.join(REPO_ROOT, "RealMan-main")
MANUS_PY_DIR = os.path.join(
    REPO_ROOT, "RealManus-LEAPHand-main", "Bidex_Manus_Teleop", "python"
)

for path in [REALMAN_DIR, MANUS_PY_DIR]:
    if path not in sys.path:
        sys.path.insert(0, path)

from teleoperate import ViveToRMMapper  # noqa: E402
from leap_hand_utils.dynamixel_client import DynamixelClient  # noqa: E402
import leap_hand_utils.leap_hand_utils as lhu  # noqa: E402


class RealSenseRGBCollector:
    def __init__(
        self,
        serial_number=None,
        width=640,
        height=480,
        fps=30,
        timeout_ms=2000,
        warmup_frames=15,
    ):
        if rs is None:
            raise RuntimeError(
                "RealSense logging requested, but pyrealsense2 is not installed. "
                "Install it with: pip install pyrealsense2"
            )

        self.serial_number = serial_number
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.timeout_ms = int(timeout_ms)
        self.warmup_frames = int(warmup_frames)
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        if self.serial_number:
            self.config.enable_device(self.serial_number)
        self.config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.rgb8,
            self.fps,
        )

        self.profile = self.pipeline.start(self.config)
        device = self.profile.get_device()
        self.device_name = device.get_info(rs.camera_info.name)
        self.device_serial = device.get_info(rs.camera_info.serial_number)

        color_profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intrinsics = color_profile.get_intrinsics()
        self.width = int(intrinsics.width)
        self.height = int(intrinsics.height)
        self.fx = float(intrinsics.fx)
        self.fy = float(intrinsics.fy)
        self.ppx = float(intrinsics.ppx)
        self.ppy = float(intrinsics.ppy)
        self.distortion_model = str(intrinsics.model)
        self.distortion_coeffs = np.asarray(intrinsics.coeffs, dtype=np.float64)

        for _ in range(max(0, self.warmup_frames)):
            try:
                self.pipeline.wait_for_frames(timeout_ms=self.timeout_ms)
            except RuntimeError:
                break

    def wait_for_frame(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=self.timeout_ms)
        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError("RealSense returned no color frame")

        return {
            "frame_number": int(color_frame.get_frame_number()),
            "timestamp_ms": float(color_frame.get_timestamp()),
            "capture_time_s": time.time(),
            "image_rgb": np.asanyarray(color_frame.get_data()).copy(),
        }

    def stop(self):
        if getattr(self, "pipeline", None) is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None


class TeleopHDF5Logger:
    """
    Streams teleoperation samples and synchronized RGB images into an HDF5 file.
    """

    def __init__(self, output_path, args, camera):
        if h5py is None:
            raise RuntimeError(
                "HDF5 logging requested, but h5py is not installed. "
                "Install it with: pip install h5py"
            )

        self.output_path = output_path
        self.sample_count = 0
        self.flush_every = max(1, int(args.log_flush_every))
        self.camera_height = int(camera.height)
        self.camera_width = int(camera.width)
        self.sample_dtype = np.dtype(
            [
                ("monotonic_s", np.float64),
                ("wall_time_s", np.float64),
                ("raw_pose", np.float64, (6,)),
                ("bounded_pose", np.float64, (6,)),
                ("safe_pose", np.float64, (6,)),
                ("smoothed_pose", np.float64, (6,)),
                ("hold_flag", np.bool_),
                ("canfd_status", np.int32),
                ("manus_joints", np.float64, (20,)),
                ("leap_pose", np.float64, (16,)),
                ("has_glove_data", np.bool_),
                ("camera_frame_number", np.int64),
                ("camera_timestamp_ms", np.float64),
                ("camera_capture_time_s", np.float64),
                ("has_camera_frame", np.bool_),
            ]
        )

        log_dir = os.path.dirname(self.output_path)
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        self.file = h5py.File(self.output_path, "w")
        self.file.require_group("time")
        self.file.require_group("arm")
        self.file.require_group("hand")
        self.file.require_group("camera")

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
        self.file.attrs["camera_serial"] = camera.device_serial
        self.file.attrs["camera_name"] = camera.device_name
        self.file.attrs["camera_width"] = self.camera_width
        self.file.attrs["camera_height"] = self.camera_height
        self.file.attrs["camera_fps"] = camera.fps
        self.file.attrs["camera_fx"] = camera.fx
        self.file.attrs["camera_fy"] = camera.fy
        self.file.attrs["camera_ppx"] = camera.ppx
        self.file.attrs["camera_ppy"] = camera.ppy
        self.file.attrs["camera_distortion_model"] = camera.distortion_model
        self.file.attrs["camera_distortion_coeffs"] = camera.distortion_coeffs
        self.file.attrs["script"] = "combined_realsense_teleop.py"
        self.file.attrs["sample_layout"] = (
            "Pose signals are grouped under /time, /arm, /hand, camera images under "
            "/camera, and per-sample metadata at /samples"
        )

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
            "camera/frame_number": self._create_dataset("camera/frame_number", (0,), (None,), np.int64),
            "camera/timestamp_ms": self._create_dataset("camera/timestamp_ms", (0,), (None,), np.float64),
            "camera/capture_time_s": self._create_dataset("camera/capture_time_s", (0,), (None,), np.float64),
            "camera/has_frame": self._create_dataset("camera/has_frame", (0,), (None,), np.bool_),
            "camera/rgb": self._create_dataset(
                "camera/rgb",
                (0, self.camera_height, self.camera_width, 3),
                (None, self.camera_height, self.camera_width, 3),
                np.uint8,
                compression="gzip",
                compression_opts=4,
            ),
        }
        self.samples_dataset = self._create_dataset("samples", (0,), (None,), self.sample_dtype)
        self.file.flush()

    def _create_dataset(
        self,
        name,
        shape,
        maxshape,
        dtype,
        compression=None,
        compression_opts=None,
    ):
        kwargs = {
            "name": name,
            "shape": shape,
            "maxshape": maxshape,
            "dtype": dtype,
            "chunks": True,
        }
        if compression is not None:
            kwargs["compression"] = compression
        if compression_opts is not None:
            kwargs["compression_opts"] = compression_opts
        return self.file.create_dataset(**kwargs)

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
        camera_frame,
    ):
        index = self.sample_count
        has_camera_frame = camera_frame is not None
        camera_rgb = (
            np.asarray(camera_frame["image_rgb"], dtype=np.uint8)
            if has_camera_frame
            else np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)
        )
        camera_frame_number = int(camera_frame["frame_number"]) if has_camera_frame else -1
        camera_timestamp_ms = (
            float(camera_frame["timestamp_ms"]) if has_camera_frame else np.nan
        )
        camera_capture_time_s = (
            float(camera_frame["capture_time_s"]) if has_camera_frame else np.nan
        )

        values = {
            "time/monotonic_s": float(monotonic_s),
            "time/wall_time_s": float(wall_time_s),
            "arm/raw_pose": np.asarray(raw_pose, dtype=np.float64),
            "arm/bounded_pose": np.asarray(bounded_pose, dtype=np.float64),
            "arm/safe_pose": np.asarray(safe_pose, dtype=np.float64),
            "arm/smoothed_pose": np.asarray(smoothed_pose, dtype=np.float64),
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
            "camera/frame_number": camera_frame_number,
            "camera/timestamp_ms": camera_timestamp_ms,
            "camera/capture_time_s": camera_capture_time_s,
            "camera/has_frame": has_camera_frame,
            "camera/rgb": camera_rgb,
        }

        for name, value in values.items():
            dataset = self.datasets[name]
            new_shape = list(dataset.shape)
            new_shape[0] = index + 1
            dataset.resize(tuple(new_shape))
            dataset[index] = value

        self.samples_dataset.resize((index + 1,))
        self.samples_dataset[index] = (
            float(monotonic_s),
            float(wall_time_s),
            np.asarray(raw_pose, dtype=np.float64),
            np.asarray(bounded_pose, dtype=np.float64),
            np.asarray(safe_pose, dtype=np.float64),
            np.asarray(smoothed_pose, dtype=np.float64),
            bool(hold_flag),
            int(canfd_status),
            np.asarray(
                glove_message if glove_message is not None else [np.nan] * 20,
                dtype=np.float64,
            ),
            np.asarray(
                leap_pose if leap_pose is not None else [np.nan] * 16,
                dtype=np.float64,
            ),
            bool(glove_message is not None),
            camera_frame_number,
            camera_timestamp_ms,
            camera_capture_time_s,
            has_camera_frame,
        )

        self.sample_count += 1
        self.file.attrs["sample_count"] = self.sample_count

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


class CombinedRealSenseTeleop:
    def __init__(self, args):
        self.args = args
        self.mapper = None
        self.hand = None
        self.glove = None
        self.camera = None
        self.logger = None

    def setup(self):
        self.mapper = ViveToRMMapper(
            robot_ip=self.args.robot_ip,
            robot_port=self.args.robot_port,
        )
        self.mapper.pos_scale = self.args.arm_pos_scale
        self.mapper.rot_scale = self.args.arm_rot_scale

        self.hand = LeapHandDirectController(
            hand_port=self.args.hand_port,
            current_limit=self.args.hand_current_limit,
        )
        self.glove = ManusErgonomicsSubscriber(
            endpoint=self.args.zmq_endpoint,
            hand_side=self.args.hand_side,
        )
        self.camera = RealSenseRGBCollector(
            serial_number=self.args.camera_serial,
            width=self.args.camera_width,
            height=self.args.camera_height,
            fps=self.args.camera_fps,
            timeout_ms=self.args.camera_timeout_ms,
            warmup_frames=self.args.camera_warmup_frames,
        )

        if self.args.control_hz > self.args.camera_fps:
            print(
                f"Warning: control_hz ({self.args.control_hz:.1f}) is higher than "
                f"camera_fps ({self.args.camera_fps}). Sample rate will be camera-limited."
            )

        if self.args.log_hdf5:
            log_path = self.args.log_path or self._default_log_path()
            self.logger = TeleopHDF5Logger(log_path, self.args, self.camera)
            print(f"HDF5 logging enabled: {log_path}")

        initial_pose = self.mapper.get_current_robot_pose()
        if initial_pose:
            time.sleep(1)

        self.mapper.calibrate(countdown=self.args.calibration_countdown)

        if self.logger is not None:
            self.logger.file.attrs["tracker_key"] = getattr(self.mapper, "tracker_key", "")
            self.logger.file.attrs["robot_home_pose"] = np.asarray(
                self.mapper.robot_home_pose, dtype=np.float64
            )
            self.logger.file.attrs["tracker_home_T"] = np.asarray(
                self.mapper.tracker_home_T, dtype=np.float64
            )
            self.logger.file.flush()

    def _default_log_path(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(
            REPO_ROOT, "Combined", "logs", f"teleop_realsense_{timestamp}.hdf5"
        )

    def run(self):
        self.setup()
        print(
            f"\nCombined teleop with RealSense running at up to "
            f"{self.args.control_hz:.1f} Hz. Press Ctrl+C to stop."
        )

        dt = 1.0 / self.args.control_hz
        warned_no_glove = False

        while True:
            loop_start = time.perf_counter()
            camera_frame = self.camera.wait_for_frame()
            wall_time_s = time.time()

            raw_pose = self.mapper.compute_target_pose()
            bounded_pose = self.mapper.apply_safety_bounds(raw_pose)
            safe_pose = self.mapper.apply_joint_limits(bounded_pose)

            if safe_pose is not None:
                final_pose = safe_pose
                is_holding = False
            else:
                final_pose = self.mapper.last_filtered_pose or self.mapper.robot_home_pose
                is_holding = True

            smooth_pose = self.mapper.apply_low_pass_filter(final_pose)
            arm_ret = self.mapper.robot.rm_movep_canfd(smooth_pose, True, 1, 50)
            if arm_ret != 0:
                if self.logger is not None:
                    self.logger.append_sample(
                        monotonic_s=loop_start,
                        wall_time_s=wall_time_s,
                        raw_pose=raw_pose,
                        bounded_pose=bounded_pose,
                        safe_pose=final_pose,
                        smoothed_pose=smooth_pose,
                        hold_flag=is_holding,
                        canfd_status=arm_ret,
                        glove_message=self.glove.message,
                        leap_pose=self.hand.last_command,
                        camera_frame=camera_frame,
                    )
                print(f"\nCANFD transmission error: {arm_ret}")
                break

            glove_message = self.glove.message
            leap_pose = self.hand.last_command
            if glove_message is not None:
                warned_no_glove = False
                leap_pose = self.hand.send_manus_command(glove_message)
            elif not warned_no_glove:
                print("\nWaiting for MANUS ergonomics data on ZMQ...")
                warned_no_glove = True

            if self.logger is not None:
                self.logger.append_sample(
                    monotonic_s=loop_start,
                    wall_time_s=wall_time_s,
                    raw_pose=raw_pose,
                    bounded_pose=bounded_pose,
                    safe_pose=final_pose,
                    smoothed_pose=smooth_pose,
                    hold_flag=is_holding,
                    canfd_status=arm_ret,
                    glove_message=glove_message,
                    leap_pose=leap_pose,
                    camera_frame=camera_frame,
                )

            camera_status = (
                f"cam:{camera_frame['frame_number']}"
                if camera_frame is not None
                else "cam:wait"
            )
            if is_holding:
                print(
                    "\r[WARNING] Arm boundary/joint limit active. "
                    f"Hand still streaming. {camera_status}   ",
                    end="",
                    flush=True,
                )
            else:
                formatted_pose = " ".join(f"{val:.4f}" for val in smooth_pose)
                glove_state = "hand:on" if glove_message is not None else "hand:wait"
                print(
                    f"\rArm: {formatted_pose}   {glove_state}   {camera_status}   ",
                    end="",
                    flush=True,
                )

            elapsed = time.perf_counter() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def shutdown(self):
        if self.logger is not None:
            self.logger.close()
        if self.camera is not None:
            self.camera.stop()
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
        description="Combined teleop: Vive tracker arm + MANUS direct-copy LEAP Hand + RealSense RGB logging"
    )
    parser.add_argument("--robot-ip", default="192.168.1.18")
    parser.add_argument("--robot-port", type=int, default=8080)
    parser.add_argument("--zmq-endpoint", default="tcp://localhost:8000")
    parser.add_argument("--hand-side", choices=["left", "right"], default="right")
    parser.add_argument("--hand-port", default=None)
    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument("--calibration-countdown", type=int, default=3)
    parser.add_argument("--arm-pos-scale", type=float, default=1.0)
    parser.add_argument("--arm-rot-scale", type=float, default=1.0)
    parser.add_argument("--hand-current-limit", type=int, default=350)
    parser.add_argument("--camera-serial", default=None)
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--camera-timeout-ms", type=int, default=2000)
    parser.add_argument("--camera-warmup-frames", type=int, default=15)
    parser.add_argument("--log-hdf5", action="store_true")
    parser.add_argument("--log-path", default=None)
    parser.add_argument("--log-flush-every", type=int, default=10)
    return parser


def main():
    args = build_parser().parse_args()
    teleop = CombinedRealSenseTeleop(args)
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
