#!/usr/bin/env python3

__author__ = "Juyeon Kim"
__email__ = "juyeon.kim@etu.sorbonne-universite.fr"
__status__ = "In development"
__version__ = ""

import rospy
from sensor_msgs.msg import LaserScan
import time
import threading
import numpy as np

class LidarPublisher:
    
    def __init__(self):
        
        # Initiailise the Lidar Publisher node
        rospy.init_node('Lidar Publisher')
        
        # Initilalise parameters
        self.pubilsh_rate = 12 
        self.min_range = 0.2
        self.max_range = 12 
        pass