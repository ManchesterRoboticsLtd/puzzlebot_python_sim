import time
from lib import puzz_msgs

class DriveStraight():
    def __init__(self):
        self.servo_angle = 70

        self.d_wall = 0.4

        self.laser_dist = self.d_wall

        self.K = 12.0

        self.omega = 4.0

        self.t_start = time.time()
    
    def spin(self,topics):

        t_total = time.time() - self.t_start

        if "LaserDistance" in topics:
            self.laser_dist = topics["LaserDistance"].data

        d_err = self.laser_dist - self.d_wall

        cmdR = puzz_msgs.Float32()
        cmdL = puzz_msgs.Float32()

        cmdR.data = self.omega - self.K*d_err
        cmdL.data = self.omega + self.K*d_err

        topics["VelocitySetR"] = cmdR
        topics["VelocitySetL"] = cmdL

        msg_servo = puzz_msgs.Float32()
        msg_servo.data = self.servo_angle
        topics["ServoAngle"]=msg_servo


        if t_total>60.0:
            topics["IsDone"] = True

        return topics
