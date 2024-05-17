#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 13 10:15:33 2024

@author: HABKI Oumaima
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class NavigationController:
    def __init__(self):
        rospy.init_node('navigation_controller')

        # Subscribe to robot's position
        rospy.Subscriber('/robot_pose', PoseStamped, self.pose_callback)

        # Publishers to activate/deactivate functionalities
        self.line_following_pub = rospy.Publisher('/line_following/activate', Bool, queue_size=1)
        self.line_detection_pub = rospy.Publisher('/line_detection/activate', Bool, queue_size=1)
        self.obstacle_avoider_pub = rospy.Publisher('/obstacle_avoider/activate', Bool, queue_size=1)
        self.corridor_pub = rospy.Publisher('/corridor/activate', Bool, queue_size=1)
        self.doors_pub = rospy.Publisher('/doors/activate', Bool, queue_size=1)

        # Coordinates of checkpoints
        self.checkpoint1_x = 0.87
        self.checkpoint1_y = -1.74
        self.checkpoint2_x = 1.0
        self.checkpoint2_y = 1.5
        self.checkpoint3_x = -1.70
        self.checkpoint3_y = - 0.2
        
    def pose_callback(self, msg):
        # Extract robot's position
        robot_x = msg.pose.position.x
        robot_y = msg.pose.position.y
        
        # Check if robot is within range of checkpoints
        if ((robot_x >=self.checkpoint1_x) or (robot_y >=self.checkpoint1_y)) and ((robot_x < self.checkpoint2_x) or (robot_y < self.checkpoint2_y)):
            # Activate line following, line detection, and obstacle avoidance
            self.line_following_pub.publish(Bool(True))
            self.line_detection_pub.publish(Bool(True))
            self.obstacle_avoider_pub.publish(Bool(True))
            # Deactivate corridor gazebo
            self.corridor_gazebo_pub.publish(Bool(False))
        elif (robot_x >= self.checkpoint2_x) and (robot_y >= self.checkpoint2_y):
            # Deactivate line following, line detection, and obstacle avoidance
            self.line_following_pub.publish(Bool(False))
            self.line_detection_pub.publish(Bool(False))
            self.obstacle_avoider_pub.publish(Bool(False))
            # Activate corridor 
            self.corridor_pub.publish(Bool(True))
	elif(robot_x >= self.checkpoint3_x) and (robot_y >= self.checkpoint3_y) and (self.corridor_gazebo_pub.publish == True): 
	    # Deactivate corridor, line following, line detection, and obstacle avoidance
            self.line_following_pub.publish(Bool(False))
            self.line_detection_pub.publish(Bool(False))
            self.obstacle_avoider_pub.publish(Bool(False))
            self.corridor_pub.publish(Bool(False))
            # Activate doors
            self.doors_pub.publish(Bool(True))
	
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = NavigationController()
    controller.run()
