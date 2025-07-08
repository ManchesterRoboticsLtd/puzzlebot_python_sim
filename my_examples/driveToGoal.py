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

        # Target points        
        self.target_x = [1,  1,  0,  0]
        self.target_y = [0, -1, -1,  0]

        self.Dmin = 0.05
        self.Kd = 1.0
        self.Kt = 1.0
        
        self.v_max = 0.2
        self.w_max = 2

        self.vc = 0.0
        self.wc = 0.0

        self.current_point = 0

    
    def spin(self,topics):

        self.dead_reckon.spin(topics)

        est_pose = self.dead_reckon.pose

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
                
            if abs(err_theta)>0.2:
                self.vc = 0

            self.w_setR = (self.vc + self.wc*self.dead_reckon.L/2) / self.dead_reckon.R
            self.w_setL = (self.vc - self.wc*self.dead_reckon.L/2) / self.dead_reckon.R
            
            if err_d<self.Dmin:
                self.current_point = self.current_point + 1                
        else:
            self.vc = 0.0
            self.wc = 0.0
            self.w_setR = 0.0                       
            self.w_setL = 0.0 
            topics["IsDone"] = True

        # Publish wheel velocities
        msg_w_setR = puzz_msgs.Float32()
        msg_w_setL = puzz_msgs.Float32()
        msg_w_setR.data = self.w_setR
        msg_w_setL.data = self.w_setL

        topics["VelocitySetR"] = msg_w_setR
        topics["VelocitySetL"] = msg_w_setL

        return topics

