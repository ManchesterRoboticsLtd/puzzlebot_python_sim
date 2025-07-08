import time
import math
from lib import my_math
from lib import puzz_msgs
import numpy as np
from matplotlib import pyplot as plt

class BinaryMapping():
    def __init__(self):
        self.pose = [0, 0, 0]

        self.map_size = [100, 100]      # map grid size

        self.map_res = 0.1              # map resolution (0.1 m)
        self.map_corner = [-5, -5]      # map lower left corner

        self.map = np.zeros(self.map_size)  # init map

        self.laser_scan = puzz_msgs.LaserScan()
        
        self.t_start = time.time()
    
    def spin(self,topics):

        # Read robot pose from topics
        if "Pose" in topics:
            self.pose = topics["Pose"].pose

        # Read lidar scan from topics
        if "LidarScan" in topics:
            self.laser_scan = topics["LidarScan"]

        # generate angle for each measurement angle = angle_res*i
        angle = self.laser_scan.angle_min
        if len(self.laser_scan.ranges)>0:
            angle_res = (self.laser_scan.angle_max - self.laser_scan.angle_min)/len(self.laser_scan.ranges)
        else:
            angle_res = 0

        for range in self.laser_scan.ranges:
            if range<0.99*self.laser_scan.range_max:
                # Compute x and y for each Lidar reading in world frame
                dx = range*math.cos(self.pose[2]+angle)
                dy = range*math.sin(self.pose[2]+angle)
                mx = self.pose[0] + dx
                my = self.pose[1] + dy

                # Change mx and my to indecies in the grid (rows and columns)
                col = math.floor((mx-self.map_corner[0])/self.map_res)
                row = math.floor((my-self.map_corner[1])/self.map_res)
            
                # check if measurement is inside grid and update cell(row,col)
                if row>=0 and row<self.map_size[0] and col>=0 and col<self.map_size[1]:
                     self.map[row][col] = 1
            
            angle = angle + angle_res

        topics["Map"] = self.map

        return topics

    def PlotMap(self):
        cmap = plt.get_cmap('Blues')
        plt.pcolor(self.map,cmap=cmap,edgecolors='k', linewidths=1)
        plt.show()
