import argparse
from datetime import datetime
import importlib
import multiprocessing as mp
from multiprocessing import shared_memory
import os
from queue import Empty, Full
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

DEFAULT_CAMERA_WIDTH = 640
DEFAULT_CAMERA_HEIGHT = 480
DEFAULT_CAMERA_FPS = 30

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


class RealSenseCamera:
    """RGB-only capture wrapper for Intel RealSense with IR Emitter explicitly disabled."""
    def __init__(self, width=640, height=480, fps=30):
        if rs is None:
            raise RuntimeError("pyrealsense2 is not installed.")

        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)

        self.pipeline = rs.pipeline()
        config = rs.config()
        
        # ONLY enable color stream to save USB bandwidth
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        
        # Start pipeline
        profile = self.pipeline.start(config)

        # THE CRITICAL FIX: Explicitly disable the IR Emitter to prevent Vive Tracker blinding
        device = profile.get_device()
        for sensor in device.query_sensors():
            if sensor.is_depth_sensor():
                if sensor.supports(rs.option.emitter_enabled):
                    sensor.set_option(rs.option.emitter_enabled, 0) # 0 = Off

    def get_camera_obs(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            return None

        return {
            "timestamp_ns": time.time_ns(),
            "color": np.asanyarray(color_frame.get_data()),
        }

    def close(self):
        if getattr(self, "pipeline", None) is not None:
            self.pipeline.stop()
            self.pipeline = None

# --- Shared Memory Utilities ---
def create_shm(name, size):
    try:
        shm = shared_memory.SharedMemory(name=name, create=True, size=size)
    except FileExistsError:
        shm = shared_memory.SharedMemory(name=name)
        shm.unlink()
        shm = shared_memory.SharedMemory(name=name, create=True, size=size)
    return shm

class CameraProcess(mp.Process):
    """Writes RGB camera frames directly to Shared Memory at 30Hz."""
    def __init__(self, stop_event, ts_value, width=640, height=480, fps=30):
        super().__init__(daemon=True)
        self.stop_event = stop_event
        self.ts_value = ts_value
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)

        self.color_size = self.width * self.height * 3

    def run(self):
        camera = None
        color_shm = None
        try:
            color_shm = create_shm('cam_color_shm', self.color_size)
            color_arr = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=color_shm.buf)

            camera = RealSenseCamera(width=self.width, height=self.height, fps=self.fps)
            
            while not self.stop_event.is_set():
                obs = camera.get_camera_obs()
                if obs is None:
                    continue

                # Write directly to memory without locking (tearing risk is minimal for RGB logs)
                np.copyto(color_arr, obs["color"])
                self.ts_value.value = obs["timestamp_ns"]

        finally:
            if camera is not None:
                camera.close()
            if color_shm is not None:
                color_shm.close()
                color_shm.unlink()


class HDF5LoggingProcess(mp.Process):
    """Dedicated process for HDF5 disk I/O. Pulls from SHM and queues without blocking teleop."""
    def __init__(self, log_queue, stop_event, output_path, args, width, height):
        super().__init__(daemon=True)
        self.log_queue = log_queue
        self.stop_event = stop_event
        self.output_path = output_path
        self.args = args
        self.width = width
        self.height = height

    def run(self):
        if h5py is None:
            print("[WARNING] h5py not installed. Logging Process exiting.")
            return

        dir_path = os.path.dirname(self.output_path)
        if dir_path:
            os.makedirs(dir_path, exist_ok=True)

        file = h5py.File(self.output_path, "w")
        flush_every = max(1, int(self.args.log_flush_every))
        sample_count = 0
        last_logged_cam_ts = 0

        color_shm = None
        local_color = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Basic metadata
        file.attrs["created_utc"] = datetime.utcnow().isoformat() + "Z"
        file.attrs["robot_ip"] = self.args.robot_ip
        file.attrs["control_hz"] = self.args.control_hz
        
        datasets = {
            "time/monotonic_s": file.create_dataset("time/monotonic_s", (0,), maxshape=(None,), dtype=np.float64, chunks=True),
            "arm/safe_pose": file.create_dataset("arm/safe_pose", (0, 6), maxshape=(None, 6), dtype=np.float64, chunks=True),
            "arm/smoothed_pose": file.create_dataset("arm/smoothed_pose", (0, 6), maxshape=(None, 6), dtype=np.float64, chunks=True),
            "hand/manus_joints": file.create_dataset("hand/manus_joints", (0, 20), maxshape=(None, 20), dtype=np.float64, chunks=True),
            "hand/leap_pose": file.create_dataset("hand/leap_pose", (0, 16), maxshape=(None, 16), dtype=np.float64, chunks=True),
            "camera/timestamp_ns": file.create_dataset("camera/timestamp_ns", (0,), maxshape=(None,), dtype=np.uint64, chunks=True),
        }

        if self.args.enable_camera:
            datasets["camera/color"] = file.create_dataset(
                "camera/color", (0, self.height, self.width, 3), maxshape=(None, self.height, self.width, 3),
                dtype=np.uint8, chunks=(1, self.height, self.width, 3), compression="lzf"
            )

            # Wait for CameraProcess to initialize shared memory
            while not self.stop_event.is_set():
                try:
                    color_shm = shared_memory.SharedMemory(name='cam_color_shm')
                    break
                except FileNotFoundError:
                    time.sleep(0.1)
            
            if color_shm:
                color_arr = np.ndarray((self.height, self.width, 3), dtype=np.uint8, buffer=color_shm.buf)

        try:
            while True:
                try:
                    data = self.log_queue.get(timeout=0.5)
                    if data is None:  # Sentinel value for shutdown
                        break
                except Empty:
                    if self.stop_event.is_set():
                        break
                    continue

                cam_ts = data.get("camera_ts", 0)
                
                # Copy from SHM only if it's a new frame to avoid redundant disk writes
                if self.args.enable_camera and color_shm and cam_ts != last_logged_cam_ts and cam_ts != 0:
                    np.copyto(local_color, color_arr)
                    last_logged_cam_ts = cam_ts

                # Resize and write datasets
                idx = sample_count
                for name, value in data["arrays"].items():
                    ds = datasets[name]
                    ds.resize((idx + 1, *ds.shape[1:]))
                    ds[idx] = value

                if self.args.enable_camera and color_shm:
                    ds_c = datasets["camera/color"]
                    ds_ts = datasets["camera/timestamp_ns"]
                    
                    ds_c.resize((idx + 1, *ds_c.shape[1:]))
                    ds_ts.resize((idx + 1,))
                    
                    ds_c[idx] = local_color
                    ds_ts[idx] = cam_ts

                sample_count += 1
                if sample_count % flush_every == 0:
                    file.flush()

        finally:
            file.attrs["sample_count"] = sample_count
            file.flush()
            file.close()
            if color_shm is not None:
                color_shm.close()


class ManusErgonomicsSubscriber:
    """Reads MANUS ergonomics stream."""
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
    """Minimal LEAP Hand controller."""
    def __init__(self, hand_port=None, current_limit=350):
        if DynamixelClient is None or lhu is None:
            raise RuntimeError("leap_hand_utils not installed.")
        self.curr_lim = current_limit
        self.motors = list(range(16))
        self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
        self.last_command = np.array(self.curr_pos, dtype=np.float64)

        port_candidates = [hand_port] if hand_port else []
        port_candidates.extend(["/dev/ttyUSB0", "/dev/ttyUSB1", "COM13"])

        self.dxl_client = None
        for candidate in port_candidates:
            try:
                self.dxl_client = DynamixelClient(self.motors, candidate, 4000000)
                self.dxl_client.connect()
                self.port = candidate
                break
            except Exception:
                self.dxl_client = None

        if self.dxl_client is None:
            raise RuntimeError("Failed to connect to LEAP Hand on any port.")

        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)

    def send_manus_command(self, hand_joints):
        pose = np.deg2rad(
            hand_joints[4:8] + [hand_joints[8] + 10] + hand_joints[9:16] +
            [90 - 1.75 * hand_joints[1]] + [-45 + 3.0 * hand_joints[0]] +
            [-30 + 3.0 * hand_joints[2]] + [hand_joints[3]]
        )
        pose[0] = -2.5 * pose[0] + np.deg2rad(20)
        pose[1] = 1.5 * pose[1]
        pose[4] = -2.5 * pose[4] + np.deg2rad(30)
        pose[5] = 1.5 * pose[5]
        pose[8] = -2.5 * pose[8]
        pose[9] = 1.5 * pose[9]
        pose[12] = 1.5 * pose[12]
        pose[13] = 1.5 * pose[13] + np.deg2rad(90)
        
        leap_pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.curr_pos = np.array(leap_pose)
        self.last_command = self.curr_pos.copy()
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
        return self.last_command
    def disconnect(self):
        """Aggressively release the serial port to prevent Zombie USB locks."""
        if self.dxl_client is not None:
            try:
                # 1. Turn off torque so motors don't freeze the bus
                self.dxl_client.set_torque_enabled(self.motors, False)
                time.sleep(0.1)
                
                # 2. Standard disconnect
                self.dxl_client.disconnect()
            except Exception:
                pass
            
            # 3. Aggressive Serial Port Release
            try:
                if hasattr(self.dxl_client, 'portHandler'):
                    self.dxl_client.portHandler.closePort()
            except Exception:
                pass


class HighFrequencyInterpolator:
    def __init__(self, alpha_pos=0.15, alpha_rot=0.15):
        self.current_pose = None
        self.alpha_pos = alpha_pos
        self.alpha_rot = alpha_rot

    def step(self, target_pose):
        target = np.array(target_pose, dtype=np.float64)
        if self.current_pose is None:
            self.current_pose = target.copy()
            return self.current_pose.tolist()

        self.current_pose[:3] += self.alpha_pos * (target[:3] - self.current_pose[:3])
        
        t = float(np.clip(self.alpha_rot, 0.0, 1.0))
        if t <= 0.0: return self.current_pose.tolist()
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
        self.interpolator = HighFrequencyInterpolator()
        self._last_cmd_pose = None
        self._last_recovery_t = 0.0

        # Subprocess Management
        self.sys_stop_event = mp.Event()
        
        # THE IPC FIX: Use RawValue to completely bypass Python's GIL and Mutex locks
        self.cam_ts_val = mp.RawValue('Q', 0)
        
        self.log_queue = mp.Queue(maxsize=5000)
        self.camera_proc = None
        self.logger_proc = None

    def _compute_raw_pose(self):
        current_T = self.mapper.get_current_tracker_matrix()
        T_delta = np.linalg.inv(self.mapper.tracker_home_T) @ current_T
        pos_delta = T_delta[:3, 3]

        remapped_pos = np.array([-pos_delta[1], -pos_delta[0], -pos_delta[2]], dtype=np.float64) * self.mapper.pos_scale
        rotvec_delta = R.from_matrix(T_delta[:3, :3]).as_rotvec()
        remapped_rotvec = np.array([-rotvec_delta[1], -rotvec_delta[0], -rotvec_delta[2]], dtype=np.float64) * self.mapper.rot_scale
        euler_delta = R.from_rotvec(remapped_rotvec).as_euler("xyz", degrees=False)

        target_pose = np.asarray(self.mapper.robot_home_pose, dtype=np.float64).copy()
        target_pose[:3] += remapped_pos
        target_pose[3:] += euler_delta
        return target_pose.tolist()

    def _move_robot_to_start_joints(self):
        target_joints = list(self.START_JOINT_DEG)
        robot = self.mapper.robot
        print("\nMoving arm to start joints (deg): " + " ".join(f"{v:.1f}" for v in target_joints))
        
        for method_name, args in [("rm_movej", (target_joints, 20, 0, 0, 1)), ("rm_movej_p", (target_joints, 20, 0, 1))]:
            method = getattr(robot, method_name, None)
            if method:
                try:
                    if method(*args) == 0:
                        time.sleep(0.5)
                        return
                except Exception:
                    continue

    def setup(self):
        self.mapper = ViveToRMMapper(robot_ip=self.args.robot_ip, robot_port=self.args.robot_port)
        self.mapper.pos_scale = self.args.arm_pos_scale
        self.mapper.rot_scale = self.args.arm_rot_scale

        self._move_robot_to_start_joints()
        
        # Strict Hardware Requirement for the LEAP Hand
        self.hand = LeapHandDirectController(hand_port=self.args.hand_port, current_limit=self.args.hand_current_limit)
        print("LEAP Hand connected successfully.")
            
        self.glove = ManusErgonomicsSubscriber(endpoint=self.args.zmq_endpoint, hand_side=self.args.hand_side)

        log_path = self.args.log_path or os.path.join(REPO_ROOT, "Combined", "logs", f"teleop_{datetime.now().strftime('%Y%m%d_%H%M%S')}.hdf5")
        
        # Start Subprocesses
        if self.args.enable_camera:
            self.camera_proc = CameraProcess(self.sys_stop_event, self.cam_ts_val, DEFAULT_CAMERA_WIDTH, DEFAULT_CAMERA_HEIGHT, DEFAULT_CAMERA_FPS)
            self.camera_proc.start()

        if self.args.log_hdf5:
            self.logger_proc = HDF5LoggingProcess(self.log_queue, self.sys_stop_event, log_path, self.args, DEFAULT_CAMERA_WIDTH, DEFAULT_CAMERA_HEIGHT)
            self.logger_proc.start()
            print(f"HDF5 Async Logger started: {log_path}")

        time.sleep(1)
        self.mapper.calibrate(countdown=self.args.calibration_countdown)
        self._last_cmd_pose = list(self.mapper.robot_home_pose)

    def run(self):
        self.setup()
        print(f"\nCombined teleop running at {self.args.control_hz:.1f} Hz. Press Ctrl+C to stop.")
        dt = 1.0 / self.args.control_hz

        while True:
            loop_start = time.perf_counter()
            
            raw_pose = self._compute_raw_pose()
            safe_pose = self.mapper.apply_safety_bounds(raw_pose) or self.mapper.last_filtered_pose or self.mapper.robot_home_pose
            smoothed_pose = self.interpolator.step(safe_pose)

            # Send Command
            arm_ret = self.mapper.robot.rm_movep_canfd(smoothed_pose, True, 1, 20)

            # Process Glove/Hand
            glove_message = self.glove.message
            leap_pose = self.hand.last_command if self.hand else None
            
            if glove_message is not None and self.hand is not None:
                leap_pose = self.hand.send_manus_command(glove_message)

            # Lightweight Logging Push
            if self.logger_proc and self.logger_proc.is_alive():
                log_data = {
                    "camera_ts": self.cam_ts_val.value,
                    "arrays": {
                        "time/monotonic_s": np.float64(loop_start),
                        "arm/safe_pose": np.asarray(safe_pose, dtype=np.float64),
                        "arm/smoothed_pose": np.asarray(smoothed_pose, dtype=np.float64),
                        "hand/manus_joints": np.asarray(glove_message if glove_message else [np.nan]*20, dtype=np.float64),
                        "hand/leap_pose": np.asarray(leap_pose if leap_pose is not None else [np.nan]*16, dtype=np.float64),
                    }
                }
                try:
                    self.log_queue.put_nowait(log_data)
                except Full:
                    pass # Drop frame instead of lagging the arm

            # Spin Wait
            while time.perf_counter() < loop_start + dt:
                pass

    def shutdown(self):
        print("\nInitiating safe teardown...")
        self.sys_stop_event.set()
        
        if self.logger_proc:
            self.log_queue.put(None) # Sentinel
            self.logger_proc.join(timeout=2.0)
            
        if self.camera_proc:
            self.camera_proc.join(timeout=2.0)
            
        if self.glove: self.glove.close()
        if self.mapper:
            try: self.mapper.robot.rm_delete_robot_arm()
            except: pass
        print("Disconnected.")

def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="192.168.1.18")
    parser.add_argument("--robot-port", type=int, default=8080)
    parser.add_argument("--zmq-endpoint", default="tcp://localhost:8000")
    parser.add_argument("--hand-side", choices=["left", "right"], default="right")
    parser.add_argument("--hand-port", default=None)
    parser.add_argument("--control-hz", type=float, default=125.0)
    parser.add_argument("--calibration-countdown", type=int, default=3)
    parser.add_argument("--arm-pos-scale", type=float, default=1.0)
    parser.add_argument("--arm-rot-scale", type=float, default=1.0)
    parser.add_argument("--hand-current-limit", type=int, default=350)
    parser.add_argument("--log-hdf5", action="store_true", default=True)
    parser.add_argument("--log-path", default=None)
    parser.add_argument("--log-flush-every", type=int, default=50)
    parser.add_argument("--enable-camera", action="store_true", default=False)
    return parser

def main():
    mp.set_start_method('spawn', force=True) # Ensure clean memory space on Windows/Linux
    args = build_parser().parse_args()
    teleop = CombinedSimpleTeleop(args)
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.shutdown()

if __name__ == "__main__":
    main()