import time
import math
from lib import puzz_msgs
from my_examples import dead_reckoning as dr
from my_examples import kalman
from my_examples import BinaryMapping as mapping
from lib import my_math
import numpy as np

class DriveToGoal():
    def __init__(self):

        self.dead_reckon = dr.DeadReckoning()
        self.kalman = kalman.Kalman()

        self.mapping = mapping.BinaryMapping()

        # Target points        
        self.target_x = [4]
        self.target_y = [0]

        self.Dmin = 0.05
        self.Kd = 1.0
        self.Kt = 1.0
        
        self.v_max = 0.2
        self.w_max = 2

        self.vc = 0.0
        self.wc = 0.0

        self.obs_dist = 0.5
        self.w_turn = 1
        self.laser_dist = 2.0

        self.state = "GO"

        self.current_point = 0

    
    def spin(self,topics):

        self.dead_reckon.spin(topics)

        est_pose = self.dead_reckon.pose

        if "LaserDistance" in topics:
            self.laser_dist = topics["LaserDistance"].data

        if self.state == "GO":
            if self.current_point<len(self.target_x):
        
                err_x = self.target_x[self.current_point] - est_pose[0]
                err_y = self.target_y[self.current_point] - est_pose[1]
                err_d = math.sqrt(err_x**2+err_y**2)
        
                err_theta = math.atan2(err_y,err_x)-est_pose[2]
                err_theta = my_math.wrap_to_pi(err_theta)
            
                self.vc = self.Kd*err_d
                self.wc = self.Kt*err_theta
            
                if self.vc>self.v_max:
                    self.vc = self.v_max
                              
                if abs(self.wc)>self.w_max:
                    self.wc = np.sign(self.wc)*self.w_max
                
                #if abs(err_theta)>0.2:
                #    self.vc = 0
                
                if err_d<self.Dmin:
                    self.current_point = self.current_point + 1   

                if self.laser_dist<self.obs_dist:
                    servo = puzz_msgs.Float32()
                    servo.data = -70
                    topics["ServoAngle"]=servo
                    self.state = "TURN"             
            else:
                self.vc = 0.0
                self.wc = 0.0
                topics["IsDone"] = True
       
        if self.state == "TURN":
            self.vc = 0.5*self.v_max
            self.wc = self.w_turn

            if self.laser_dist>self.obs_dist:
                servo = puzz_msgs.Float32()
                servo.data = 0.0
                topics["ServoAngle"]=servo

                self.state = "GO"             
            
        self.w_setR = (self.vc + self.wc*self.dead_reckon.L/2) / self.dead_reckon.R
        self.w_setL = (self.vc - self.wc*self.dead_reckon.L/2) / self.dead_reckon.R
            

        # Publish wheel velocities
        msg_w_setR = puzz_msgs.Float32()
        msg_w_setL = puzz_msgs.Float32()
        msg_w_setR.data = self.w_setR
        msg_w_setL.data = self.w_setL

        topics["VelocitySetR"] = msg_w_setR
        topics["VelocitySetL"] = msg_w_setL

        return topics

