import time
import math
from lib import my_math
from lib import puzz_msgs
import numpy as np

class DeadReckoning():
    def __init__(self):
        self.pose = [0, 0, 0]

        self.sigma_squared = 0.1

        self.w_r = 0.0
        self.w_l = 0.0

        self.R = 0.05
        self.L = 0.18

        self.Sig = np.array([[0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0]])
        
        self.t_start = time.time()
    
    def spin(self,topics):

        dt = time.time() - self.t_start
        self.t_start = time.time()

        # Read wheel angular velocities from topics (msg type is Float32)
        if "VelocityEncR" in topics:
            self.w_r = topics["VelocityEncR"].data
        if "VelocityEncL" in topics:
            self.w_l = topics["VelocityEncL"].data
        if "Pose" in topics:
            self.pose = topics["Pose"].pose
            self.Sig = topics["Pose"].cov

        # Compute linear and angular velocities of the robot
        self.Vr = (self.w_r+self.w_l)*self.R/2
        self.Wr = (self.w_r-self.w_l)*self.R/self.L

        # Update pose mean
        self.pose[0] = self.pose[0] + dt*self.Vr*math.cos(self.pose[2])
        self.pose[1] = self.pose[1] + dt*self.Vr*math.sin(self.pose[2])
        self.pose[2] = my_math.wrap_to_pi(self.pose[2] + dt*self.Wr)

        # Update pose covariance matrix
        H = np.array([[1, 0, -dt*self.Vr*math.sin(self.pose[2])],
                      [0, 1, dt*self.Vr*math.cos(self.pose[2])],
                      [0, 0, 1]])
             
        dH = np.array([[0.5*dt*self.R*math.cos(self.pose[2]), 0.5*dt*self.R*math.cos(self.pose[2])],
                       [0.5*dt*self.R*math.sin(self.pose[2]), 0.5*dt*self.R*math.sin(self.pose[2])],
                       [dt*self.R/self.L, -dt*self.R/self.L]])
                       
        K = np.array([[self.sigma_squared*abs(self.w_r),   0                                     ],
                      [0,                             self.sigma_squared*abs(self.w_l)]])
                      
        Q = dH @ K @ dH.T                  # Q = dH*K*dH'
        
        self.Sig = H @ self.Sig @ H.T + Q  # Sig = H*Sig*H' + Q

        # Publish dead-reckoning pose and covariance
        msg_pose = puzz_msgs.Pose()
        msg_pose.pose = self.pose
        msg_pose.cov = self.Sig

        topics["Pose"] = msg_pose

        return topics

