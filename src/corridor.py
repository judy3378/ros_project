#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Oumaima HABKI

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class CorridorNavigator:
    def __init__(self):
        rospy.init_node('corridor_navigator')
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Robot movement parameters
        self.linear_speed = 0.2  # Adjust as needed
        self.angular_speed = 0.3  # Adjust as needed
        self.min_distance = 0.18 # Adjust as needed
        self.max_distance = 0.35  # Adjust as needed 
        self.state = 'follow_left_wall'
        self.completed_mission = False
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
        self.run()

    def scan_callback(self, scan_msg):
        # Process laser scan data
        left_distances = scan_msg.ranges[:50]
        right_distances = scan_msg.ranges[-50:]

        left_distance = min(x for x in left_distances if not np.isinf(x)) if left_distances else self.max_distance
        right_distance = min(x for x in right_distances if not np.isinf(x)) if right_distances else self.max_distance

        # Robot navigation logic
        if self.state == 'follow_left_wall':
            if left_distance > self.max_distance:
                self.move(0, self.angular_speed)
            elif left_distance < self.min_distance:
                self.move(0, -self.angular_speed)
            else:
                self.move(self.linear_speed, 0)
                if right_distance < self.max_distance:
                    self.state = 'follow_right_wall'
        elif self.state == 'follow_right_wall':
            if right_distance > self.max_distance:
                self.move(0, -self.angular_speed)
            elif right_distance < self.min_distance:
                self.move(0, self.angular_speed)
            else:
                self.move(self.linear_speed, 0)
                if left_distance < self.max_distance:
                    self.state = 'follow_left_wall'
        if left_distance >= self.max_distance and right_distance >= self.max_distance:
            self.completed_mission = True
            self.move(0, 0)
            rospy.signal_shutdown("Robot has reached the end of the corridor")


    def move(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        robot = CorridorNavigator()
        while not rospy.is_shutdown() and not robot.completed_mission:
            robot.move(robot.linear_speed, 0)
    except rospy.ROSInterruptException:
        pass
