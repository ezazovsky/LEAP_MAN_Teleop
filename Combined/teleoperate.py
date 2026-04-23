import time
import math
import os
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REALMAN_DIR = os.path.join(REPO_ROOT, "RealMan-main")
if REALMAN_DIR not in sys.path:
    sys.path.insert(0, REALMAN_DIR)

# Vive Imports
from track import ViveTrackerModule
from fairmotion_ops import conversions, math as fairmotion_math

# RM Robot Imports
from Robotic_Arm.rm_robot_interface import *


class ViveToRMMapper:
    def __init__(self, robot_ip="192.168.1.18", robot_port=8080):
        # 1. Initialize Vive Tracker
        print("Initializing Vive Tracker...")
        self.vive_module = ViveTrackerModule()
        devices = self.vive_module.return_selected_devices("tracker")
        if not devices:
            raise RuntimeError("No Vive Trackers found!")
        self.tracker_key = list(devices.keys())[0]
        self.tracker = devices[self.tracker_key]
        
        # Base station origin config
        self.base_station_origin = conversions.p2T(np.array([3.0, -2.8, -3.0]))
        self.origin_inv = fairmotion_math.invertT(self.base_station_origin)

        # 2. Initialize RM Robot
        print(f"Connecting to RM Robot at {robot_ip}:{robot_port}...")
        self.robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        self.handle = self.robot.rm_create_robot_arm(robot_ip, robot_port)
        if self.handle.id == -1:
            raise RuntimeError("Failed to connect to RM Robot.")
            
        self.robot.rm_set_arm_run_mode(1)

        # Calibration state
        self.tracker_home_T = None
        self.robot_home_pose = None
        
        # Scaling factors
        self.pos_scale = 1.0
        self.rot_scale = 1.0

        # State tracking for fallback and velocity clamping
        self.last_filtered_pose = None

        # ==========================================
        # --- SAFETY BOUNDS CONFIGURATION ---
        # ==========================================
        
        # 1. Cartesian Limits (meters)
        # RM65 Specific Safety Envelope
        self.safe_x = [-0.37, 0.37]    # Meters
        self.safe_y = [-0.37, 0.37]    
        self.safe_z = [0.05, 0.4]     # Stay 5cm off table, 5cm below max height

        # Critical: RM65 reach is 0.61m. We MUST bound it to 0.55m to avoid singularities.
        self.max_reach_radius = 0.58
        self.min_reach_radius = 0.15   # Don't let it punch itself in the base
        # Start damping motion before the hard radius wall to avoid kinematic lockups.
        self.soft_reach_radius = 0.54
        self.soft_reach_gain = 0.08
        self.max_radial_step = 0.035  # m/update near singular boundary

        # 3. Maximum Tracker Jump limits (Glitch Protection)
        self.max_pos_jump = 0.1
        self.max_rot_jump = 0.85  # rad/update, shortest-path metric
        # RM65 J4-J6 are rated around 225 deg/s; keep a margin for stable teleop.
        self.max_rot_speed_rad_s = math.radians(225.0)
        self.rot_speed_safety_factor = 0.75
        # Smooth rotational commanding to avoid freeze-then-snap behavior.
        self.max_rot_accel_rad_s2 = math.radians(450.0)
        self._last_safety_filter_time = None
        self._rot_rate_cmd = np.zeros(3, dtype=np.float64)

        # Orientation singularity damping (used only near outer workspace boundary).
        self.boundary_rot_damp_gain = 0.25

        # Shoulder singularity references from RM65 documentation (degrees).
        self.shoulder_singularity_center_deg = np.array([0.0, 43.4, -105.7], dtype=np.float64)
        self.shoulder_singularity_tol_deg = np.array([14.0, 18.0, 18.0], dtype=np.float64)
        self.shoulder_singularity_j5_targets_deg = np.array([-30.0, 62.3], dtype=np.float64)
        self.shoulder_singularity_j5_tol_deg = 22.0
        # Disabled by default to keep the teleop loop responsive.
        self.enable_joint_singularity_guard = False
        self.joint_guard_check_interval_s = 0.08
        self._last_joint_guard_check_t = 0.0
        self._cached_shoulder_zone_state = False

    @staticmethod
    def _wrap_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    @classmethod
    def _shortest_angle_diff(cls, target, current):
        return cls._wrap_angle(target - current)

    def _is_shoulder_singularity_zone(self):
        if not self.enable_joint_singularity_guard:
            return False

        now = time.perf_counter()
        if now - self._last_joint_guard_check_t < self.joint_guard_check_interval_s:
            return self._cached_shoulder_zone_state

        self._last_joint_guard_check_t = now
        res, joints = self.robot.rm_get_joint_degree()
        if res != 0 or joints is None or len(joints) < 6:
            self._cached_shoulder_zone_state = False
            return False

        j = np.asarray(joints, dtype=np.float64)
        primary_error = np.abs(j[:3] - self.shoulder_singularity_center_deg)
        primary_match = np.all(primary_error <= self.shoulder_singularity_tol_deg)
        if not primary_match:
            self._cached_shoulder_zone_state = False
            return False

        j5_match = np.any(
            np.abs(j[4] - self.shoulder_singularity_j5_targets_deg)
            <= self.shoulder_singularity_j5_tol_deg
        )
        self._cached_shoulder_zone_state = bool(j5_match)
        return self._cached_shoulder_zone_state

    def get_current_robot_pose(self):
        res, joint_angles = self.robot.rm_get_joint_degree()
        if res == 0:
            return self.robot.rm_algo_forward_kinematics(joint_angles, 1)
        return None

    def get_current_tracker_matrix(self):
        return self.origin_inv @ self.tracker.get_T()

    def calibrate(self, countdown=3):
        print(f"\nCalibrating in {countdown} seconds. Hold tracker steady!")
        for i in range(countdown, 0, -1):
            print(f"{i}...")
            time.sleep(1)
            
        self.tracker_home_T = self.get_current_tracker_matrix().copy()
        self.robot_home_pose = self.get_current_robot_pose()
        
        if not self.robot_home_pose:
            raise RuntimeError("Failed to get robot home pose during calibration.")
            
        self.last_filtered_pose = self.robot_home_pose

        # --- Verify home pose is within bounds before starting ---
        hp = self.robot_home_pose
        print(f"\nHome pose XYZ: {hp[0]:.3f}, {hp[1]:.3f}, {hp[2]:.3f}")
        print(f"Home pose RPY: {hp[3]:.3f}, {hp[4]:.3f}, {hp[5]:.3f}")

        assert self.safe_x[0] <= hp[0] <= self.safe_x[1], \
            f"[CALIBRATION FAIL] Home X={hp[0]:.3f} is OUTSIDE safe_x={self.safe_x}"
        assert self.safe_y[0] <= hp[1] <= self.safe_y[1], \
            f"[CALIBRATION FAIL] Home Y={hp[1]:.3f} is OUTSIDE safe_y={self.safe_y}"
        assert self.safe_z[0] <= hp[2] <= self.safe_z[1], \
            f"[CALIBRATION FAIL] Home Z={hp[2]:.3f} is OUTSIDE safe_z={self.safe_z}"

        radius = math.sqrt(hp[0]**2 + hp[1]**2 + hp[2]**2)
        assert radius <= self.max_reach_radius, \
            f"[CALIBRATION FAIL] Home radius={radius:.3f} exceeds max_reach={self.max_reach_radius}"

        print("Calibration Complete!")
        print(f"Robot Anchor: {[f'{v:.4f}' for v in self.robot_home_pose]}")

    def compute_target_pose(self):
        current_T = self.get_current_tracker_matrix()
        T_delta = np.linalg.inv(self.tracker_home_T) @ current_T
        pos_delta = T_delta[:3, 3]
        
        remapped_pos = np.array([
            -pos_delta[1],
            -pos_delta[0],
            -pos_delta[2] 
        ]) * self.pos_scale
        
        R_delta = T_delta[:3, :3]
        rotvec_delta = R.from_matrix(R_delta).as_rotvec()
        
        remapped_rotvec = np.array([
            -rotvec_delta[1], 
            -rotvec_delta[0], 
            -rotvec_delta[2]
        ]) * self.rot_scale
        
        euler_delta = R.from_rotvec(remapped_rotvec).as_euler('xyz', degrees=False)

        target_pose = list(self.robot_home_pose)
        target_pose[0] += remapped_pos[0]
        target_pose[1] += remapped_pos[1]
        target_pose[2] += remapped_pos[2]
        target_pose[3] += euler_delta[0]
        target_pose[4] += euler_delta[1]
        target_pose[5] += euler_delta[2]
        
        return target_pose

    def apply_safety_bounds(self, target_pose):
        if self.last_filtered_pose is None:
            self.last_filtered_pose = list(target_pose)
            self._last_safety_filter_time = time.perf_counter()
            self._rot_rate_cmd[:] = 0.0
            return list(target_pose)

        safe_pose = list(target_pose)
        original_pose = list(target_pose)

        # STEP 1: Jump protection FIRST — filter bad sensor data before anything else.
        # If the tracker glitches and jumps 1m, we only move max_pos_jump toward it.
        dx = safe_pose[0] - self.last_filtered_pose[0]
        dy = safe_pose[1] - self.last_filtered_pose[1]
        dz = safe_pose[2] - self.last_filtered_pose[2]
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        if dist > self.max_pos_jump:
            scale = self.max_pos_jump / dist
            safe_pose[0] = self.last_filtered_pose[0] + (dx * scale)
            safe_pose[1] = self.last_filtered_pose[1] + (dy * scale)
            safe_pose[2] = self.last_filtered_pose[2] + (dz * scale)

        # STEP 1B: Rotation jump protection (shortest-angle path per axis).
        now = time.perf_counter()
        if self._last_safety_filter_time is None:
            dt = 1.0 / 125.0
        else:
            dt = now - self._last_safety_filter_time
        if dt > 0.03:
            # Drop stale angular velocity state after transient stalls.
            self._rot_rate_cmd[:] = 0.0
        dt = max(1.0 / 250.0, min(1.0 / 90.0, dt))

        max_rate = self.max_rot_speed_rad_s * self.rot_speed_safety_factor

        raw_rot_diffs = np.array([
            self._shortest_angle_diff(safe_pose[3], self.last_filtered_pose[3]),
            self._shortest_angle_diff(safe_pose[4], self.last_filtered_pose[4]),
            self._shortest_angle_diff(safe_pose[5], self.last_filtered_pose[5]),
        ])

        # Convert desired pose delta into desired angular rate, then slew-limit rate.
        desired_rate = np.clip(raw_rot_diffs / dt, -max_rate, max_rate)
        max_rate_delta = self.max_rot_accel_rad_s2 * dt
        rate_delta = np.clip(
            desired_rate - self._rot_rate_cmd,
            -max_rate_delta,
            max_rate_delta,
        )
        self._rot_rate_cmd += rate_delta
        self._rot_rate_cmd = np.clip(self._rot_rate_cmd, -max_rate, max_rate)

        rot_diffs = self._rot_rate_cmd * dt

        # Never overshoot the commanded shortest-path delta on any axis.
        overshoot_mask = np.abs(rot_diffs) > np.abs(raw_rot_diffs)
        if np.any(overshoot_mask):
            rot_diffs[overshoot_mask] = raw_rot_diffs[overshoot_mask]
            self._rot_rate_cmd[overshoot_mask] = rot_diffs[overshoot_mask] / dt

        # Keep legacy hard jump cap as a final safety net.
        max_rot_step = min(self.max_rot_jump, max_rate * dt)

        rot_diffs = np.clip(rot_diffs, -max_rot_step, max_rot_step)

        rot_dist = np.linalg.norm(rot_diffs)
        if rot_dist > max_rot_step and rot_dist > 1e-9:
            scale = max_rot_step / rot_dist
            rot_diffs *= scale

        if np.any(np.abs(rot_diffs) > 0.0):
            safe_pose[3] = self._wrap_angle(self.last_filtered_pose[3] + rot_diffs[0])
            safe_pose[4] = self._wrap_angle(self.last_filtered_pose[4] + rot_diffs[1])
            safe_pose[5] = self._wrap_angle(self.last_filtered_pose[5] + rot_diffs[2])

        # STEP 2: Hard Cartesian box clamp.
        # Stops values at the edge of the defined safe envelope.
        safe_pose[0] = max(self.safe_x[0], min(self.safe_x[1], safe_pose[0]))
        safe_pose[1] = max(self.safe_y[0], min(self.safe_y[1], safe_pose[1]))
        safe_pose[2] = max(self.safe_z[0], min(self.safe_z[1], safe_pose[2]))

        # STEP 3: Sphere (reach radius) clamp.
        # Prevents singularities near max arm extension.
        last_radius = math.sqrt(
            self.last_filtered_pose[0]**2 + self.last_filtered_pose[1]**2 + self.last_filtered_pose[2]**2
        )
        radius = math.sqrt(safe_pose[0]**2 + safe_pose[1]**2 + safe_pose[2]**2)

        # If already near boundary, limit outward radial growth per cycle to avoid elbow lockups.
        if radius > self.soft_reach_radius and radius > last_radius:
            max_allowed = last_radius + self.max_radial_step
            if radius > max_allowed and radius > 1e-9:
                scale = max_allowed / radius
                safe_pose[0] *= scale
                safe_pose[1] *= scale
                safe_pose[2] *= scale
                radius = max_allowed

        if radius > self.soft_reach_radius and radius <= self.max_reach_radius and radius > 1e-9:
            # Soft wall: gently pull commanded point inward before the hard clamp.
            overflow = radius - self.soft_reach_radius
            softened_radius = radius - (self.soft_reach_gain * overflow)
            scale = softened_radius / radius
            safe_pose[0] *= scale
            safe_pose[1] *= scale
            safe_pose[2] *= scale

        # Shoulder singularity zone fallback: stop outward push and bias slightly inward.
        if self._is_shoulder_singularity_zone():
            radius = math.sqrt(safe_pose[0]**2 + safe_pose[1]**2 + safe_pose[2]**2)
            if radius > last_radius and radius > 1e-9:
                scale = last_radius / radius
                safe_pose[0] *= scale
                safe_pose[1] *= scale
                safe_pose[2] *= scale

            inward_target = max(self.min_reach_radius, last_radius - 0.004)
            radius = math.sqrt(safe_pose[0]**2 + safe_pose[1]**2 + safe_pose[2]**2)
            if radius > inward_target and radius > 1e-9:
                scale = inward_target / radius
                safe_pose[0] *= scale
                safe_pose[1] *= scale
                safe_pose[2] *= scale

            for idx in range(3, 6):
                desired = safe_pose[idx]
                last = self.last_filtered_pose[idx]
                diff = self._shortest_angle_diff(desired, last)
                safe_pose[idx] = self._wrap_angle(last + (0.45 * diff))

        radius = math.sqrt(safe_pose[0]**2 + safe_pose[1]**2 + safe_pose[2]**2)
        if radius > self.max_reach_radius:
            scale = self.max_reach_radius / radius
            safe_pose[0] *= scale
            safe_pose[1] *= scale
            safe_pose[2] *= scale

        # STEP 4: Minimum reach radius clamp.
        # Prevents the arm from colliding with its own base.
        radius = math.sqrt(safe_pose[0]**2 + safe_pose[1]**2 + safe_pose[2]**2)
        if radius < self.min_reach_radius and radius > 1e-9:
            scale = self.min_reach_radius / radius
            safe_pose[0] *= scale
            safe_pose[1] *= scale
            safe_pose[2] *= scale

        # STEP 5: Orientation handling.
        for idx in range(3, 6):
            safe_pose[idx] = self._wrap_angle(safe_pose[idx])

        # Near boundary singularity zone: damp orientation change, don't hard clamp it.
        radius = math.sqrt(safe_pose[0]**2 + safe_pose[1]**2 + safe_pose[2]**2)
        if radius > self.soft_reach_radius:
            boundary_ratio = (radius - self.soft_reach_radius) / max(
                1e-6, (self.max_reach_radius - self.soft_reach_radius)
            )
            boundary_ratio = float(np.clip(boundary_ratio, 0.0, 1.0))
            damp = self.boundary_rot_damp_gain * boundary_ratio
            for idx in range(3, 6):
                desired = safe_pose[idx]
                last = self.last_filtered_pose[idx]
                diff = self._shortest_angle_diff(desired, last)
                safe_pose[idx] = self._wrap_angle(last + ((1.0 - damp) * diff))

        # STEP 6: Log when clamping actually fires so you can verify bounds are working.
        tol = 1e-6
        clamped = any(abs(safe_pose[i] - original_pose[i]) > tol for i in range(6))
        if clamped:
            pos_changed = any(abs(safe_pose[i] - original_pose[i]) > tol for i in range(3))
            rot_changed = any(abs(safe_pose[i] - original_pose[i]) > tol for i in range(3, 6))
            rot_wrap_only = rot_changed and all(
                abs(self._shortest_angle_diff(safe_pose[i], original_pose[i])) <= tol
                for i in range(3, 6)
            )

            if rot_wrap_only and not pos_changed:
                clamp_kind = "ANGLE_WRAP_ONLY"
            else:
                reasons = []
                if pos_changed:
                    reasons.append("CARTESIAN")
                if rot_changed and not rot_wrap_only:
                    reasons.append("ROTATION")
                if not reasons:
                    reasons.append("MIXED")
                clamp_kind = "+".join(reasons)

            print(
                f"[BOUNDS CLAMPED:{clamp_kind}] "
                f"in=({original_pose[0]:.3f}, {original_pose[1]:.3f}, {original_pose[2]:.3f}, "
                f"{original_pose[3]:.3f}, {original_pose[4]:.3f}, {original_pose[5]:.3f}) "
                f"out=({safe_pose[0]:.3f}, {safe_pose[1]:.3f}, {safe_pose[2]:.3f}, "
                f"{safe_pose[3]:.3f}, {safe_pose[4]:.3f}, {safe_pose[5]:.3f})"
            )

        self.last_filtered_pose = safe_pose
        self._last_safety_filter_time = now
        return safe_pose