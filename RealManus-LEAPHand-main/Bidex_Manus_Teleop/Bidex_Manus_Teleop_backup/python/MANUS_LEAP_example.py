import numpy as np
import zmq
import threading
import time

##Import LEAP Hand
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu


'''
This script integrates this MANUS SDK with our LEAP Hand Python SDK.
To run this:
- Plug in gloves and run our MANUS SDK
- Plug in LEAP right hand and run this script

See our Manus SDK and LEAP Hand SDK for further details.
'''

'''
This LEAP Hand node is borrwed from the LEAP Hand Python SDK
'''
class LeapNode:
    def __init__(self):
        ####Some parameters
        self.kP = 400
        self.kI = 0
        self.kD = 300
        self.curr_lim = 350
        self.prev_pos = self.pos = self.curr_pos = lhu.allegro_to_LEAPhand(np.zeros(16))
           
        #You can put the correct port here or have the node auto-search for a hand at the first 3 ports.
        self.motors = motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        try:
            self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(motors, 'COM13', 4000000)
                self.dxl_client.connect()
        #Enables position-current control mode and the default parameters, it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kP, 84, 2) # Pgain stiffness     
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2) # Pgain stiffness for side to side should be a bit less
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kI, 82, 2) # Igain
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.kD, 80, 2) # Dgain damping
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2) # Dgain damping for side to side should be a bit less
        #Max at current (in unit 1ma) so don't overheat and grip too hard #500 normal or #350 for lite
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Receive LEAP pose and directly control the robot
    def set_leap(self, pose):
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #allegro compatibility
    def set_allegro(self, pose):
        pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #Sim compatibility, first read the sim value in range [-1,1] and then convert to leap
    def set_ones(self, pose):
        pose = lhu.sim_ones_to_LEAPhand(np.array(pose))
        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos)
    #read position
    def read_pos(self):
        return self.dxl_client.read_pos()
    #read velocity
    def read_vel(self):
        return self.dxl_client.read_vel()
    #read current
    def read_cur(self):
        return self.dxl_client.read_cur()

class ZMQSubscriber:
    """
    Creates a thread that subscribes to a ZMQ publisher
    """
    def __init__(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.PULL)
        self.socket.setsockopt(zmq.CONFLATE, True)     
        self.socket.connect("tcp://localhost:8000")
        self._subscriber_thread = threading.Thread(target=self._update_value)
        self._subscriber_thread.start()
        self._value = None
        self.last_message = None
    
    @property
    def message(self):
        return self._value
    #This thread runs in the background and receives the messages
    def _update_value(self):
        while True:
            message = self.socket.recv()
            message = message.decode('utf-8')
            message = message.split(",") 
            if len(message) == 40:
                self._value = list(map(float,message[20:40]))  #Get the right hand data (second half of the 40 datapoints coming in)


'''
Note we are only copying joint angles here.  

The thumb will never be this good in this mode due to the difference in kinematics between the human and robot
'''
if __name__ == "__main__":
    zmq_sub = ZMQSubscriber()
    leap_node = LeapNode()
    while True:
        if zmq_sub.message is None:
            print("No data from gloves")
        else:
            right = zmq_sub.message          
            right = np.deg2rad(right[4:8] + [right[8] + 10] + right[9:16] +[90-1.75*right[1]] + [-45 + 3.0*right[0]] + [-30+3.0*right[2]] + [right[3]])
            right[0] = -2.5 * right[0]
            right[1] = 1.5 * right[1]
            right[4] = -2.5 * right[4]
            right[5] = 1.5 * right[5]
            right[8] = -2.5 * right[8] - np.deg2rad(45)
            right[9] = 1.5 * right[9]
            right[12] = 1.5 * right[12]
            right[13] = 1.5 * right[13] + np.deg2rad(90)
            leap_node.set_allegro(right)
        time.sleep(0.01)
