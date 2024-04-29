#!/usr/bin/env python3
# -*- coding: utf-8 -*-
__author__ = "Juyeon Kim"
# Date: 2024

#%% IMPORTS
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, LaserScan
from cv_bridge  import CvBridge
#from control_bot.msg import SpeedDirection

#%% CLASS
class LaneFollowing():
    """ Class that handles the lane following challenge.
        It uses the sensor data(image and ___) to determine the speed and direction of the robot."""
    def __init__(self):
        # initialize the node
        rospy.loginfo("[INFO] -- Initializing the lane following node")
        rospy.init_node("lane_class", anonymous=True)
        
        # Subscirbe to the sensor data
        rospy.Subscriber("lidar_data", LaserScan, self.callback_lidar)
        #rospy.Subscriber("camera_info", CameraInfo, self.callback_camera)
        
        # init the publisher
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        
        # initialize the stored data
        self.lidar_data = LaserScan()
        #self.camera_info = CameraInfo() is 
        
        # Conditions
        self.wrong_way = False      # True if the robot is going the wrong way
        
        #