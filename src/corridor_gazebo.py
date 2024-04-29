#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author : Juyeon Kim
# Date: from 2023. In development 2024

"""This code functions once the robot is in the beginnning of the corridor on the red line"""

import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan 
import numpy as np

class CorridorNavigator:
    def __init__(self):
        # Initialize ROS node and subscribers
        rospy.init_node('Corridor_Gazebo')
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Set movement speed
        self.linear_speed = 0.2
        self.angular_speed = 0.3

        # Set distance thresholds
        self.min_distance = 0.2
        self.max_distance = 0.55

        # Set initial state
        self.state = 'follow_left_wall'
        print("init")

        # Keep track of mission completion
        self.completed_mission = False
        # Initialize publisher and start moving robot
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.run()


    def scan_callback(self, scan_msg):
        # Get left and right distances from LDS excluding infinte distances
        left_distances = [x for x in scan_msg.ranges[:50]if not np.isinf(x)]
        if len(left_distances) >0:
            left_distance = min(left_distances)
            #print("leftdist",left_distance)
        else:
            left_distance = self.max_distance
            #print("leftdist=maxdist")
        
        right_distances = [x for x in scan_msg.ranges[-50:]if not np.isinf(x)]
        if len(right_distances) >0:
            right_distance = min(right_distances)
            #print("rightdist",right_distance)
        else:
            right_distance = self.max_distance
            #print("rightdist=max_dist")
            

        if self.state == 'follow_left_wall':
            if left_distance > self.max_distance:
                # Turn left to get closer to left wall
                self.move(0, self.angular_speed)
                #print("tooclose to right, turn left")
                
            elif left_distance < self.min_distance:
                # Turn right to avoid collision with left wall
                self.move(0, -self.angular_speed)
                #print("tooclose to left turn right")
            else:
                # Move forward along left wall
                self.move(self.linear_speed, 0)

                if right_distance < self.max_distance:
                    #print("right distance ", right_distance)
                    #print("max dist", self.max_distance)
                    
                    # Switch to following right wall
                    self.state = 'follow_right_wall'
                    #print("change state to right")

        elif self.state == 'follow_right_wall':
            if right_distance > self.max_distance:
                # Turn right to get closer to right wall
                
                self.move(0, -self.angular_speed)
                #print("tooclose to left turn right")
            elif right_distance < self.min_distance:
                # Turn left to avoid collision with right wall
                self.move(0, self.angular_speed)
                #print("tooclose to right, turn left")
            else:
                # Move forward along right wall
                self.move(self.linear_speed, 0)

                if left_distance < self.max_distance:
                    # Switch to following left wall
                    
                    #print("left distance ", left_distance)
                    #print("max dist", self.max_distance)
                    self.state = 'follow_left_wall'
                    #print("change state to left")
            
        if left_distance >=self.max_distance and right_distance >= self.max_distance:
            # Stop robot and mission
            
            #print("ld {}, max {}; rd {}".format(left_distance, self.max_distance, right_distance))
            self.completed_mission = True
            self.move(0, 0)
            rospy.signal_shutdown("Robot has reached the end of the corridor")

    def move(self, linear, angular):
        # Move robot using Twist message
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def run(self):
        # Start moving robot
        #self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.spin()

if __name__ == '__main__':
    try:
        # Initialize node and start moving robot
        #rospy.init_node('corridor_navigator')
        robot = CorridorNavigator()

        # Move robot forward until it reaches the end of the corridor
        while not rospy.is_shutdown()and not robot.completed_mission:
            robot.move(robot.linear_speed, 0)

        # Stop robot
        #robot.move(0, 0)

    except rospy.ROSInterruptException:
        pass
