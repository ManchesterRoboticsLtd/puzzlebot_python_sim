import time
from lib import puzz_msgs

class DriveSquare():
    def __init__(self):
        self.state = "Straight"

        self.x = 0.0
        self.theta = 0.0
        self.D = 1.0

        self.R = 0.05  # wheel radius
        self.L = 0.18  # robot width

        self.count_turns = 0

        self.t_start = time.time()
    
    def spin(self,topics):

        dt = time.time() - self.t_start
        self.t_start = time.time()

        if self.state == "Straight":
            self.v = 0.2
            self.w = 0.0
            self.x = self.x +dt*self.v
            if self.x > self.D:                
                if self.count_turns < 3:
                    self.state = "Turn"
                    self.theta = 0.0
                else:
                    self.state = "Done"

        if self.state == "Turn":
            self.v = 0.0
            self.w = 0.6
            self.theta = self.theta + dt*self.w
            if self.theta > 1.54:
                self.count_turns = self.count_turns + 1
                self.state = "Straight"
                self.x = 0.0

        if self.state == "Done":
            self.v = 0.0
            self.w = 0.0
            topics["IsDone"] = True

        cmdR = puzz_msgs.Float32()
        cmdL = puzz_msgs.Float32()

        cmdR.data = (self.v + self.w*self.L/2) / self.R
        cmdL.data = (self.v - self.w*self.L/2) / self.R

        topics["VelocitySetR"] = cmdR
        topics["VelocitySetL"] = cmdL

        return topics