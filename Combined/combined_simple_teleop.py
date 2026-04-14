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

        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
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
        self.file.attrs["script"] = "combined_simple_teleop.py"

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
    ):
        index = self.sample_count

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
        }

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
    Optimized strictly for 6D poses [X, Y, Z, Rx, Ry, Rz] with proper 
    shortest-path angle wrapping.
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

        # 2. Rotation Interpolation (Rx, Ry, Rz) - Shortest Path EMA
        # Step A: Calculate the raw angular difference
        angular_diff = target[3:] - self.current_pose[3:]
        
        # Step B: Wrap the difference to strictly sit between -pi and +pi.
        # Note: This assumes your RealMan robot uses radians (which is standard). 
        # If it uses degrees, change np.pi to 180 and 2 * np.pi to 360.
        shortest_path_diff = (angular_diff + np.pi) % (2 * np.pi) - np.pi
        
        # Step C: Apply the alpha smoothing to that safely wrapped difference
        self.current_pose[3:] += self.alpha_rot * shortest_path_diff
        
        # Step D: Normalize the stored current pose to keep it bounded
        # This prevents floating point drift if you spin in one direction forever
        self.current_pose[3:] = (self.current_pose[3:] + np.pi) % (2 * np.pi) - np.pi

        return self.current_pose.tolist()


class CombinedSimpleTeleop:
    def __init__(self, args):
        self.args = args
        self.mapper = None
        self.hand = None
        self.glove = None
        self.logger = None
        self.interpolator = HighFrequencyInterpolator()

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
        if self.args.log_hdf5:
            log_path = self.args.log_path or self._default_log_path()
            self.logger = TeleopHDF5Logger(log_path, self.args)
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

        while True:
            loop_start = time.perf_counter()
            wall_time_s = time.time()

            raw_pose = self.mapper.compute_target_pose()
            
            # 1. Apply Cartesian safety limits (bounding box, radius, etc.)
            # bounded_pose = self.mapper.apply_safety_bounds(raw_pose)
            
            # 2. DO NOT apply joint limits to a Cartesian pose. 
            # The RealMan controller handles IK and joint limits natively during movep.

            bounded_pose = raw_pose
            
            if bounded_pose is not None:
                final_pose = bounded_pose
                is_holding = False
            else:
                final_pose = self.mapper.last_filtered_pose or self.mapper.robot_home_pose
                is_holding = True

            # Using the new, safe EMA interpolator that properly maps rotation
            smooth_pose = self.interpolator.step(final_pose)
            
            # trajectory_mode=1 (curve fitting), radio=20 (reduced buffer)
            arm_ret = self.mapper.robot.rm_movep_canfd(smooth_pose, True, 1, 20)
            
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
                )

            if is_holding:
                print(
                    "\r[WARNING] Arm boundary limit active. Hand still streaming.   ",
                    end="",
                    flush=True,
                )
            else:
                formatted_pose = " ".join(f"{val:.4f}" for val in smooth_pose)
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
        description="Simple combined teleop: Vive tracker arm + MANUS direct-copy LEAP Hand"
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
    parser.add_argument("--log-hdf5", action="store_true")
    parser.add_argument("--log-path", default=None)
    parser.add_argument("--log-flush-every", type=int, default=50)
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