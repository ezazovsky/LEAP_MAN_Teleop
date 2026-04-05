import sys
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

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

        # --- EMA Low-Pass Filter State ---
        self.last_filtered_pose = None
        self.alpha_pos = 0.15
        self.alpha_rot = 0.05

        # ==========================================
        # --- SAFETY BOUNDS CONFIGURATION ---
        # ==========================================
        
        # 1. Cartesian Limits (meters)
        self.safe_x = [-0.4, 0.4]    
        self.safe_y = [-0.5, 0.5]    
        self.safe_z = [0.05, 0.6]    

        # 2. Spherical Reach Limits (meters)
        self.max_reach_radius = 0.56 
        self.min_reach_radius = 0.15 
        
        # 3. Maximum Tracker Jump limits (Glitch Protection)
        self.max_pos_jump = 0.036  

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
            rotvec_delta[1], 
            rotvec_delta[0], 
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
            return target_pose

        safe_pose = list(target_pose)

        # 1. Velocity Clamping: If moving too fast, limit the speed instead of dropping the frame
        dx = safe_pose[0] - self.last_filtered_pose[0]
        dy = safe_pose[1] - self.last_filtered_pose[1]
        dz = safe_pose[2] - self.last_filtered_pose[2]
        pos_dist = math.sqrt(dx**2 + dy**2 + dz**2)

        if pos_dist > self.max_pos_jump:
            scale = self.max_pos_jump / pos_dist
            safe_pose[0] = self.last_filtered_pose[0] + (dx * scale)
            safe_pose[1] = self.last_filtered_pose[1] + (dy * scale)
            safe_pose[2] = self.last_filtered_pose[2] + (dz * scale)

        # 2. Cartesian Workspace Bounding
        safe_pose[0] = max(self.safe_x[0], min(self.safe_x[1], safe_pose[0]))
        safe_pose[1] = max(self.safe_y[0], min(self.safe_y[1], safe_pose[1]))
        safe_pose[2] = max(self.safe_z[0], min(self.safe_z[1], safe_pose[2]))

        # 3. Spherical Reach Bounding
        radius = math.sqrt(safe_pose[0]**2 + safe_pose[1]**2 + safe_pose[2]**2)
        
        if radius > self.max_reach_radius:
            scale = self.max_reach_radius / radius
            safe_pose[0] *= scale
            safe_pose[1] *= scale
            safe_pose[2] *= scale
            
        elif radius < self.min_reach_radius:
             scale = self.min_reach_radius / radius
             safe_pose[0] *= scale
             safe_pose[1] *= scale
             safe_pose[2] *= scale

        return safe_pose

    def apply_joint_limits(self, target_pose):
        joint_limits = [
            [-178, 178],
            [-130, 130],
            [-135, 135],
            [-178, 178],
            [-128, 128],
            [-360, 360] 
        ]

        res, current_joints = self.robot.rm_get_joint_degree()
        if res != 0: return None 
            
        try:
            params = rm_inverse_kinematics_params_t(current_joints, target_pose, 1)
        except TypeError:
            params = rm_inverse_kinematics_params_t()
            params.q_in = current_joints
            params.q_pose = target_pose
            params.flag = 1

        res, predicted_joints = self.robot.rm_algo_inverse_kinematics(params)

        if res != 0: return None

        for i in range(6):
            if not (joint_limits[i][0] <= predicted_joints[i] <= joint_limits[i][1]):
                return None 

        return target_pose

    def apply_low_pass_filter(self, target_pose):
        if self.last_filtered_pose is None:
            self.last_filtered_pose = target_pose
            return target_pose

        filtered_pose = [0.0] * 6
        for i in range(3):
            filtered_pose[i] = (self.alpha_pos * target_pose[i]) + ((1.0 - self.alpha_pos) * self.last_filtered_pose[i])
            
        for i in range(3, 6):
            diff = target_pose[i] - self.last_filtered_pose[i]
            diff = (diff + math.pi) % (2 * math.pi) - math.pi
            filtered_pose[i] = self.last_filtered_pose[i] + (self.alpha_rot * diff)

        self.last_filtered_pose = filtered_pose
        return filtered_pose

def main():
    try:
        mapper = ViveToRMMapper()
        
        initial_pose = mapper.get_current_robot_pose()
        if initial_pose:
            safe_pose = list(initial_pose)
            time.sleep(1)

        mapper.calibrate()

        print("\nStreaming data via CANFD... Press Ctrl+C to stop.")
        dt = 0.02  # 50 Hz loop
        
        while True:
            loop_start = time.perf_counter()
            
            # 1. Compute Raw Pose
            raw_pose = mapper.compute_target_pose()
            
            # 2. Apply Bounds (Now dynamically limits speed and distance)
            bounded_pose = mapper.apply_safety_bounds(raw_pose)
            
            # 3. Check Joint Limits via IK
            safe_pose = mapper.apply_joint_limits(bounded_pose)
            
            # --- THE HEARTBEAT FIX ---
            if safe_pose is not None:
                final_pose = safe_pose
                is_holding = False
            else:
                # IK Failed/Out of bounds! Feed the stream our last good pose instead of giving up.
                final_pose = mapper.last_filtered_pose or mapper.robot_home_pose
                is_holding = True

            # 4. Apply Smoothing Filter (Smoothly eases into the boundary hold)
            smooth_pose = mapper.apply_low_pass_filter(final_pose)
            
            # 5. Send to Robot - NEVER SKIP THIS CALL!
            ret = mapper.robot.rm_movep_canfd(smooth_pose, True, 1, 50)
            if ret != 0:
                print(f"\nCANFD transmission error: {ret}")
                break
            
            # 6. UI Update
            if is_holding:
                print("\r[WARNING] Boundary/Joint Limit! Holding pos.   ", end="", flush=True)
            else:
                formatted_pose = " ".join([f"{val:.4f}" for val in smooth_pose])
                print(f"\rTarget: {formatted_pose}       ", end="", flush=True)

            # Enforce 50Hz timing
            elapsed = time.perf_counter() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nTeleoperation stopped by user.")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        if 'mapper' in locals():
            mapper.robot.rm_delete_robot_arm()
            print("Disconnected.")

if __name__ == "__main__":
    main()