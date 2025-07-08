import time
import math
from lib import my_math
from lib import puzz_msgs
from my_examples import dead_reckoning as dr
import numpy as np

class Kalman():
    def __init__(self):
        self.dead_reckon = dr.DeadReckoning()
        self.pose = [0, 0, 0]

        self.map = [[1, 1.6,-1],
                    [2, 2.6,-2],
                    [3, -2,1],
                    [4, -1,-2]]

        self.markers = []

        self.Sig = np.array([[0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0],
                             [0.0, 0.0, 0.0]])
        
        self.cov = 0.1
        
        self.t_start = time.time()
    
    def spin(self,topics):

        dt = time.time() - self.t_start
        self.t_start = time.time()

        self.dead_reckon.spin(topics)

        self.pose = self.dead_reckon.pose
        self.Sig = self.dead_reckon.Sig

        # Read wheel angular velocities from topics (msg type is Float32)
        if "markers" in topics:
            self.markers = topics["markers"].markers

        for marker in self.markers:
            # Check if the marker from measurements is found in the map
            found = False
            M = [0,0]
            for mark in self.map:
                if mark[0] == marker.id:
                    found = True
                    M[0] = mark[1]
                    M[1] = mark[2]
        
            if found:
            
                dx = M[0] - self.pose[0]
                dy = M[1] - self.pose[1]
                
                p = dx**2 + dy**2
                
                Z_hat = np.array([ math.sqrt(p), my_math.wrap_to_pi(math.atan2(dy,dx)-self.pose[2])])
        
                G = np.array([[-dx/math.sqrt(p),  -dy/math.sqrt(p), 0],
                              [ dy/p,             -dx/p,           -1]])
                      
                R = np.array([[self.cov,   0],
                              [  0, self.cov]])
                      
                diff = [marker.range - Z_hat[0], my_math.wrap_to_pi(marker.theta - Z_hat[1])]
                
                Z = G @ self.Sig @ G.T + R               # Z = G*Sig*G' + R
        
                K = self.Sig @ G.T @ np.linalg.inv(Z)    # Kalman gain:  K = Sig*G'*inv(Z)
        
                self.pose = self.pose + K @ diff         # Update pose mean:  s = s + K*(Z-Z_hat)
        
                self.Sig = (np.eye(3)-K @ G) @ self.Sig  # Update covariance:  Sig = (I-K*G)*Sig

        # Publish dead-reckoning pose and covariance
        msg_pose = puzz_msgs.Pose()
        msg_pose.pose = self.pose
        msg_pose.cov = self.Sig

        topics["Pose"] = msg_pose

        return topics

