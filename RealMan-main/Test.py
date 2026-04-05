import math
import time
from Robotic_Arm.rm_robot_interface import *

# 1. Initialize and Connect
robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle = robot.rm_create_robot_arm("192.168.1.18", 8080)

if handle.id == -1:
    print("Failed to connect.")
    exit()

robot.rm_set_arm_run_mode(1)

def get_current_pose():
    """Calculates Cartesian pose using joint angles"""
    res, joint_angles = robot.rm_get_joint_degree()
    if res == 0:
        return robot.rm_algo_forward_kinematics(joint_angles, 1) # 1 = Euler angles
    return None

def generate_human_dataset(start_pose, duration, dt):
    """
    Pre-computes a dataset of poses mimicking a human 'sweeping' motion.
    Uses a velocity envelope to ensure organic acceleration/deceleration.
    """
    dataset = []
    steps = int(duration / dt)
    
    for i in range(steps):
        t = i * dt
        t_rel = t / duration  # Normalized time: 0.0 to 1.0
        
        # The Envelope: math.sin(pi * t_rel)^2 creates a bell curve
        # Starts exactly at 0 multiplier, peaks at 1 in the middle, ends at 0
        envelope = math.sin(math.pi * t_rel)**2
        
        # Calculate organic offsets for multiple axes
        dx = 0.05 * math.sin(4 * math.pi * t_rel) * envelope  # Slight forward/back 
        dy = 0.15 * math.sin(2 * math.pi * t_rel) * envelope  # Wide 15cm left/right sweep
        dz = 0.03 * math.sin(2 * math.pi * t_rel) * envelope  # Slight vertical arc
        drz = 0.40 * math.sin(2 * math.pi * t_rel) * envelope # Wrist turning naturally
        
        # Apply offsets to the anchor pose
        pose = list(start_pose)
        pose[0] += dx
        pose[1] += dy
        pose[2] += dz
        pose[5] += drz  # Index 5 is Rz (Yaw in radians)
        
        dataset.append(pose)
        
    return dataset

try:
    # 2. Pre-Move Safety Buffer
    initial_pose = get_current_pose()
    if initial_pose:
        safe_pose = list(initial_pose)
        safe_pose[2] += 0.05  # Move UP 5cm to clear the base
        print("Lifting to a safe starting height...")
        robot.rm_movel(safe_pose, 20, 0, 0, 1)
        time.sleep(1)
    
    # 3. Capture Anchored Starting Position
    start_pose = get_current_pose()
    if not start_pose:
        print("Error: Could not retrieve current pose.")
        exit()
    
    # 4. Generate the "Sample Data"
    duration = 8.0   # 8 seconds for the full sweeping motion
    dt = 0.02        # 50 Hz data resolution
    
    print("Synthesizing human motion dataset...")
    trajectory_data = generate_human_dataset(start_pose, duration, dt)

    # 5. Stream the Dataset in Real-Time
    print("Streaming data via CANFD...")
    start_time = time.perf_counter()

    for target_pose in trajectory_data:
        loop_start = time.perf_counter()
        
        # Stream the current frame
        ret = robot.rm_movep_canfd(target_pose, True, 1, 50)
        
        if ret != 0:
            print(f"CANFD transmission interrupted. Error code: {ret}")
            break
            
        # Maintain strict 50Hz timing
        elapsed = time.perf_counter() - loop_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    print("Data stream execution completed smoothly.")

finally:
    robot.rm_delete_robot_arm()
    print("Disconnected.")