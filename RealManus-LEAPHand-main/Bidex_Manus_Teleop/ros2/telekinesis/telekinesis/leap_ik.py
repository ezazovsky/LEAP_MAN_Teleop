#!/usr/bin/env python3
import pybullet as p
import numpy as np
import rclpy
import os
import pybullet_data 

from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
import sys
from ament_index_python.packages import get_package_share_directory

class LeapPybulletIK(Node):
    def __init__(self):
        super().__init__('leap_pyb_ik')  
        p.connect(p.GUI)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81) 
        p.setTimeStep(1./240.)

        self.planeId = p.loadURDF("plane.urdf", [0, 0, -0.3])

        path_src = os.path.abspath(__file__)
        path_src = os.path.dirname(path_src)
        self.is_left = self.declare_parameter('isLeft', False).get_parameter_value().bool_value
        self.glove_to_leap_mapping_scale = 1.6
        self.leapEndEffectorIndex = [3, 4, 8, 9, 13, 14, 18, 19]
        
        # 1. DEFINE HAND ORIGIN
        # We store this so we can calculate offsets relative to it.
        if self.is_left:
            self.hand_origin = [-0.05, -0.03, -0.25] 
        else:
            self.hand_origin = [-0.05, -0.03, -0.125]

        if self.is_left:
            path_src = os.path.join(path_src, "leap_hand_mesh_left/robot_pybullet.urdf")
            self.LeapId = p.loadURDF(
                path_src,
                self.hand_origin, # Use variable
                p.getQuaternionFromEuler([0, 1.57, 1.57]), 
                useFixedBase = True
            )
            self.cubeId = p.loadURDF("cube_small.urdf", [0.1, 0.0, 0.05])   
            self.pub_hand = self.create_publisher(JointState, '/leaphand_node/cmd_allegro_left', 10)
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/l_short", self.get_glove_data, 10)
        else:
            path_src = os.path.join(path_src, "leap_hand_mesh_right/robot_pybullet.urdf")
            self.LeapId = p.loadURDF(
                path_src,
                self.hand_origin, # Use variable
                p.getQuaternionFromEuler([0, 1.57, 1.57]), 
                useFixedBase = True
            )
            
            self.cubeId = p.loadURDF("cube_small.urdf", [0.1, 0.0, 0.05]) 
            self.pub_hand = self.create_publisher(JointState, '/leaphand_node/cmd_allegro_right', 10)
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/r_short", self.get_glove_data, 10)

        p.changeDynamics(self.cubeId, -1, mass=0.1, lateralFriction=1.0)
        self.numJoints = p.getNumJoints(self.LeapId)
        p.setRealTimeSimulation(0)
        self.create_target_vis()
            
    def create_target_vis(self):
        small_ball_radius = 0.01
        small_ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=small_ball_radius)
        ball_radius = 0.01
        ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
        baseMass = 0.001
        # Start balls at wrist position instead of arbitrary [0.25, 0.25, 0]
        basePosition = self.hand_origin 
        
        self.ballMbt = []
        for i in range(0,4):
            # Keyword arguments to be safe
            self.ballMbt.append(p.createMultiBody(baseMass=baseMass, baseCollisionShapeIndex=ball_shape, basePosition=basePosition))    
            p.setCollisionFilterGroupMask(self.ballMbt[i], -1, 0, 0)
        p.changeVisualShape(self.ballMbt[0], -1, rgbaColor=[1, 0, 0, 1]) 
        p.changeVisualShape(self.ballMbt[1], -1, rgbaColor=[0, 1, 0, 1]) 
        p.changeVisualShape(self.ballMbt[2], -1, rgbaColor=[0, 0, 1, 1])  
        p.changeVisualShape(self.ballMbt[3], -1, rgbaColor=[1, 1, 1, 1])
        
    def update_target_vis(self, hand_pos):
        _, current_orientation = p.getBasePositionAndOrientation( self.ballMbt[0])
        p.resetBasePositionAndOrientation(self.ballMbt[0], hand_pos[3], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[1])
        p.resetBasePositionAndOrientation(self.ballMbt[1], hand_pos[2], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[2])
        p.resetBasePositionAndOrientation(self.ballMbt[2], hand_pos[7], current_orientation)
        _, current_orientation = p.getBasePositionAndOrientation(self.ballMbt[3])
        p.resetBasePositionAndOrientation(self.ballMbt[3], hand_pos[1], current_orientation)
        
    def get_glove_data(self, pose):
        poses = pose.poses
        hand_pos = []  
        for i in range(0,10):
            # --- PRESERVE ORIGINAL BEHAVIOR IN A LOCAL FRAME ---
            
            # 1. Calculate the position using the ORIGINAL logic (Absolute World Coords)
            orig_world_x = poses[i].position.x * self.glove_to_leap_mapping_scale * 1.15
            orig_world_y = poses[i].position.y * self.glove_to_leap_mapping_scale
            orig_world_z = -poses[i].position.z * self.glove_to_leap_mapping_scale
            
            # 2. Convert to LOCAL OFFSET (Relative to Wrist)
            # Local = Original_World - Wrist_Origin
            local_x = orig_world_x - self.hand_origin[0]
            local_y = orig_world_y - self.hand_origin[1]
            local_z = orig_world_z - self.hand_origin[2]
            
            # 3. Apply to Wrist (New Relative Logic)
            # World = Wrist_Origin + Local
            # This ensures the result is identical to step 1, but now "attached" to the wrist.
            final_x = self.hand_origin[0] + local_x
            final_y = self.hand_origin[1] + local_y
            final_z = self.hand_origin[2] + local_z
            
            hand_pos.append([final_x, final_y, final_z])

        # Apply offsets (Original logic kept intact)
        hand_pos[4][1] = hand_pos[4][1] + 0.002
        hand_pos[6][1] = hand_pos[6][1] + 0.002
        
        self.compute_IK(hand_pos)
        self.update_target_vis(hand_pos)
        
    def compute_IK(self, hand_pos):
        p.stepSimulation()      

        leapEndEffectorPos = [
            hand_pos[2], hand_pos[3],
            hand_pos[4], hand_pos[5],
            hand_pos[6], hand_pos[7],
            hand_pos[0], hand_pos[1]
        ]

        jointPoses = p.calculateInverseKinematics2(
            self.LeapId,
            self.leapEndEffectorIndex,
            leapEndEffectorPos,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )
        
        combined_jointPoses = (jointPoses[0:4] + (0.0,) + jointPoses[4:8] + (0.0,) + jointPoses[8:12] + (0.0,) + jointPoses[12:16] + (0.0,))
        combined_jointPoses = list(combined_jointPoses)

        for i in range(20):
            p.setJointMotorControl2(
                bodyIndex=self.LeapId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=combined_jointPoses[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

        real_robot_hand_q = np.array([float(0.0) for _ in range(16)])
        real_robot_hand_q[0:4] = jointPoses[0:4]
        real_robot_hand_q[4:8] = jointPoses[4:8]
        real_robot_hand_q[8:12] = jointPoses[8:12]
        real_robot_hand_q[12:16] = jointPoses[12:16]
        real_robot_hand_q[0:2] = real_robot_hand_q[0:2][::-1]
        real_robot_hand_q[4:6] = real_robot_hand_q[4:6][::-1]
        real_robot_hand_q[8:10] = real_robot_hand_q[8:10][::-1]
        stater = JointState()
        stater.position = [float(i) for i in real_robot_hand_q]
        self.pub_hand.publish(stater)

def main(args=None):
    rclpy.init(args=args)
    leappybulletik = LeapPybulletIK()
    rclpy.spin(leappybulletik)
    leappybulletik.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()