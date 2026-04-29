"""
robot_pose_controller.py

Standalone controller: accepts a 6D Cartesian arm pose and 16-joint LEAP hand pose,
applies safety bounds, and sends commands to the real hardware.
"""

import math
import importlib
import os
import sys
import time
import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_DIR = os.path.dirname(__file__)
REALMAN_DIR = os.path.join(REPO_ROOT, "RMAPI", "Python")
MANUS_PY_DIR = os.path.join(REPO_ROOT, "LMAPI", "python")

for _p in [SRC_DIR, REALMAN_DIR, MANUS_PY_DIR]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e  # noqa: E402

try:
    DynamixelClient = importlib.import_module("leap_hand_utils.dynamixel_client").DynamixelClient
    lhu = importlib.import_module("leap_hand_utils.leap_hand_utils")
except Exception:
    DynamixelClient = None
    lhu = None

class ArmSafetyFilter:
    """Stateful safety filter for 6D Cartesian poses."""
    safe_x: list = [-0.37, 0.37]
    safe_y: list = [-0.37, 0.37]
    safe_z: list = [0.05, 0.40]

    max_reach_radius: float = 0.58
    min_reach_radius: float = 0.15
    soft_reach_radius: float = 0.54
    soft_reach_gain: float = 0.08
    max_radial_step: float = 0.035

    max_pos_jump: float = 0.1
    max_rot_jump: float = 0.85
    max_rot_speed_rad_s: float = math.radians(225.0)
    rot_speed_safety_factor: float = 0.75
    max_rot_accel_rad_s2: float = math.radians(450.0)
    boundary_rot_damp_gain: float = 0.25

    def __init__(self):
        self.last_filtered_pose = None
        self._last_safety_filter_time = None
        self._rot_rate_cmd = np.zeros(3, dtype=np.float64)

    def seed(self, initial_pose):
        self.last_filtered_pose = list(initial_pose)
        self._last_safety_filter_time = time.perf_counter()
        self._rot_rate_cmd[:] = 0.0

    @staticmethod
    def _wrap_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    @classmethod
    def _shortest_angle_diff(cls, target, current):
        return cls._wrap_angle(target - current)

    def apply(self, target_pose):
        if self.last_filtered_pose is None:
            self.last_filtered_pose = list(target_pose)
            self._last_safety_filter_time = time.perf_counter()
            self._rot_rate_cmd[:] = 0.0
            return list(target_pose)

        safe_pose = list(target_pose)
        original_pose = list(target_pose)

        dx = safe_pose[0] - self.last_filtered_pose[0]
        dy = safe_pose[1] - self.last_filtered_pose[1]
        dz = safe_pose[2] - self.last_filtered_pose[2]
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist > self.max_pos_jump:
            scale = self.max_pos_jump / dist
            safe_pose[0] = self.last_filtered_pose[0] + dx * scale
            safe_pose[1] = self.last_filtered_pose[1] + dy * scale
            safe_pose[2] = self.last_filtered_pose[2] + dz * scale

        now = time.perf_counter()
        dt = (now - self._last_safety_filter_time) if self._last_safety_filter_time else 1.0 / 125.0
        if dt > 0.03:
            self._rot_rate_cmd[:] = 0.0
        dt = max(1.0 / 250.0, min(1.0 / 90.0, dt))

        max_rate = self.max_rot_speed_rad_s * self.rot_speed_safety_factor
        raw_rot_diffs = np.array([
            self._shortest_angle_diff(safe_pose[3], self.last_filtered_pose[3]),
            self._shortest_angle_diff(safe_pose[4], self.last_filtered_pose[4]),
            self._shortest_angle_diff(safe_pose[5], self.last_filtered_pose[5]),
        ])
        desired_rate = np.clip(raw_rot_diffs / dt, -max_rate, max_rate)
        max_rate_delta = self.max_rot_accel_rad_s2 * dt
        self._rot_rate_cmd += np.clip(desired_rate - self._rot_rate_cmd, -max_rate_delta, max_rate_delta)
        self._rot_rate_cmd = np.clip(self._rot_rate_cmd, -max_rate, max_rate)
        rot_diffs = self._rot_rate_cmd * dt

        overshoot = np.abs(rot_diffs) > np.abs(raw_rot_diffs)
        if np.any(overshoot):
            rot_diffs[overshoot] = raw_rot_diffs[overshoot]
            self._rot_rate_cmd[overshoot] = rot_diffs[overshoot] / dt

        max_rot_step = min(self.max_rot_jump, max_rate * dt)
        rot_diffs = np.clip(rot_diffs, -max_rot_step, max_rot_step)
        rot_dist = float(np.linalg.norm(rot_diffs))
        if rot_dist > max_rot_step and rot_dist > 1e-9:
            rot_diffs *= max_rot_step / rot_dist

        if np.any(np.abs(rot_diffs) > 0.0):
            safe_pose[3] = self._wrap_angle(self.last_filtered_pose[3] + rot_diffs[0])
            safe_pose[4] = self._wrap_angle(self.last_filtered_pose[4] + rot_diffs[1])
            safe_pose[5] = self._wrap_angle(self.last_filtered_pose[5] + rot_diffs[2])

        safe_pose[0] = max(self.safe_x[0], min(self.safe_x[1], safe_pose[0]))
        safe_pose[1] = max(self.safe_y[0], min(self.safe_y[1], safe_pose[1]))
        safe_pose[2] = max(self.safe_z[0], min(self.safe_z[1], safe_pose[2]))

        def _radius(p): return math.sqrt(p[0]**2 + p[1]**2 + p[2]**2)

        last_r = _radius(self.last_filtered_pose)
        r = _radius(safe_pose)

        if r > self.soft_reach_radius and r > last_r:
            max_allowed = last_r + self.max_radial_step
            if r > max_allowed and r > 1e-9:
                scale = max_allowed / r
                safe_pose[0] *= scale; safe_pose[1] *= scale; safe_pose[2] *= scale
                r = max_allowed

        if self.soft_reach_radius < r <= self.max_reach_radius and r > 1e-9:
            overflow = r - self.soft_reach_radius
            softened = r - self.soft_reach_gain * overflow
            scale = softened / r
            safe_pose[0] *= scale; safe_pose[1] *= scale; safe_pose[2] *= scale

        r = _radius(safe_pose)
        if r > self.max_reach_radius and r > 1e-9:
            scale = self.max_reach_radius / r
            safe_pose[0] *= scale; safe_pose[1] *= scale; safe_pose[2] *= scale

        r = _radius(safe_pose)
        if r < self.min_reach_radius and r > 1e-9:
            scale = self.min_reach_radius / r
            safe_pose[0] *= scale; safe_pose[1] *= scale; safe_pose[2] *= scale

        for idx in range(3, 6):
            safe_pose[idx] = self._wrap_angle(safe_pose[idx])

        r = _radius(safe_pose)
        if r > self.soft_reach_radius:
            boundary_ratio = float(np.clip((r - self.soft_reach_radius) / max(1e-6, self.max_reach_radius - self.soft_reach_radius), 0.0, 1.0))
            damp = self.boundary_rot_damp_gain * boundary_ratio
            for idx in range(3, 6):
                diff = self._shortest_angle_diff(safe_pose[idx], self.last_filtered_pose[idx])
                safe_pose[idx] = self._wrap_angle(self.last_filtered_pose[idx] + (1.0 - damp) * diff)

        tol = 1e-6
        if any(abs(safe_pose[i] - original_pose[i]) > tol for i in range(6)):
            pos_chg = any(abs(safe_pose[i] - original_pose[i]) > tol for i in range(3))
            rot_chg = any(abs(safe_pose[i] - original_pose[i]) > tol for i in range(3, 6))
            rot_wrap_only = rot_chg and all(abs(self._shortest_angle_diff(safe_pose[i], original_pose[i])) <= tol for i in range(3, 6))
            kind = "ANGLE_WRAP_ONLY" if (rot_wrap_only and not pos_chg) else ("+".join((["CARTESIAN"] if pos_chg else []) + (["ROTATION"] if rot_chg and not rot_wrap_only else [])) or "MIXED")
            print(f"[BOUNDS CLAMPED:{kind}] out=({safe_pose[0]:.3f},{safe_pose[1]:.3f},{safe_pose[2]:.3f},{safe_pose[3]:.3f},{safe_pose[4]:.3f},{safe_pose[5]:.3f})")

        self.last_filtered_pose = safe_pose
        self._last_safety_filter_time = now
        return safe_pose


class RobotPoseController:
    def __init__(self, robot_ip: str = "192.168.1.18", robot_port: int = 8080, hand_port: str = None, current_limit: int = 350, connect_hand: bool = True):
        print(f"Connecting to RealMan arm at {robot_ip}:{robot_port} ...")
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = self.robot.rm_create_robot_arm(robot_ip, robot_port)
        if handle.id == -1: raise RuntimeError("Failed to connect to RealMan arm.")
        self.robot.rm_set_arm_run_mode(1)
        print("Arm connected.")

        self.safety = ArmSafetyFilter()
        current_pose = self._get_current_arm_pose()
        if current_pose:
            self.safety.seed(current_pose)
        else:
            print("[WARNING] Could not read current arm pose — safety filter will self-seed on first call.")

        self._hand_motors = list(range(16))
        self.hand = None
        if connect_hand:
            if DynamixelClient is None or lhu is None: raise RuntimeError("leap_hand_utils not installed.")
            print("Connecting to LEAP hand ...")
            self.hand = self._init_hand(hand_port, current_limit)
            print("Hand connected.")

    def _get_current_arm_pose(self):
        res, joints = self.robot.rm_get_joint_degree()
        if res == 0 and joints:
            result = self.robot.rm_algo_forward_kinematics(joints, 1)
            if result: return result
        return None

    def _init_hand(self, hand_port, current_limit):
        kP, kI, kD = 400, 0, 300
        candidates = [p for p in [hand_port, "/dev/ttyUSB0", "/dev/ttyUSB1", "COM13"] if p]
        dxl = None
        for port in candidates:
            try:
                dxl = DynamixelClient(self._hand_motors, port, 4_000_000)
                dxl.connect()
                print(f"  LEAP hand found on {port}")
                break
            except Exception:
                dxl = None
        if dxl is None: raise RuntimeError(f"No LEAP hand found on candidate ports.")

        motors = self._hand_motors
        dxl.sync_write(motors, np.ones(16) * 5, 11, 1)
        dxl.set_torque_enabled(motors, True)
        dxl.sync_write(motors, np.ones(16) * kP, 84, 2)
        dxl.sync_write([0, 4, 8], np.ones(3) * (kP * 0.75), 84, 2)
        dxl.sync_write(motors, np.ones(16) * kI, 82, 2)
        dxl.sync_write(motors, np.ones(16) * kD, 80, 2)
        dxl.sync_write([0, 4, 8], np.ones(3) * (kD * 0.75), 80, 2)
        dxl.sync_write(motors, np.ones(16) * current_limit, 102, 2)
        dxl.write_desired_pos(motors, lhu.allegro_to_LEAPhand(np.zeros(16)))
        return dxl

    def send_arm_pose(self, pose_6d, trajectory_mode=1, trajectory_radio=20):
        safe = self.safety.apply(list(np.asarray(pose_6d, dtype=np.float64)))
        return self.robot.rm_movep_canfd(safe, True, trajectory_mode, trajectory_radio)

    def send_hand_pose(self, leap_joints_16d):
        if self.hand is None: return
        clipped = lhu.angle_safety_clip(np.asarray(leap_joints_16d, dtype=np.float64))
        self.hand.write_desired_pos(self._hand_motors, clipped)

    def send_combined(self, arm_pose_6d, hand_pose_16d, trajectory_mode=1, trajectory_radio=20):
        ret = self.send_arm_pose(arm_pose_6d, trajectory_mode, trajectory_radio)
        if hand_pose_16d is not None: self.send_hand_pose(hand_pose_16d)
        return ret

    def shutdown(self):
        if self.hand is not None:
            try: self.hand.disconnect()
            except Exception: pass
        try: self.robot.rm_delete_robot_arm()
        except Exception: pass
        print("RobotPoseController disconnected.")

def _build_parser():
    import argparse
    p = argparse.ArgumentParser(description="Send a single 6D pose to the robot.")
    p.add_argument("--robot-ip", default="192.168.1.18")
    p.add_argument("--robot-port", type=int, default=8080)
    p.add_argument("--hand-port", default=None)
    p.add_argument("--pose", nargs=6, type=float, metavar=("X", "Y", "Z", "Rx", "Ry", "Rz"))
    p.add_argument("--no-hand", action="store_true")
    return p

def main():
    args = _build_parser().parse_args()
    ctrl = RobotPoseController(robot_ip=args.robot_ip, robot_port=args.robot_port, hand_port=args.hand_port, connect_hand=not args.no_hand)
    try:
        if args.pose:
            print(f"Sending pose: {args.pose}")
            print(f"CANFD status: {ctrl.send_arm_pose(args.pose)}")
        else:
            print("No --pose given. Connected successfully; shutting down.")
    finally:
        ctrl.shutdown()

if __name__ == "__main__":
    main()