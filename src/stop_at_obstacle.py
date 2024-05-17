#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import math

def index_from_angle(angle, data):
    # Calculate index for a given angle using the LaserScan message fields
    return int((angle - data.angle_min) / data.angle_increment)

class StopAtObstacle:
    def __init__(self):
        rospy.init_node('stop_at_obstacle', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.obstacle_avoidance_active_pub = rospy.Publisher('/obstacle_avoidance_active', Bool, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.rate = rospy.Rate(10)

        self.min_distance = 0.25 # Increase the minimum safe distance to consider

        # Set up parameters
        self.use_sim = rospy.get_param('use_sim', True)
        self.obstacle_detected = False

    def scan_callback(self, data):
        front_range = self.get_range(data, 335, 25)  # -25 to 25 degrees

        if not self.use_sim: # In the real world
            range = []
            for nb in front_range:
                if nb == 0:
                    nb = 10
                range.append(nb)
            front_range = range

        if min(front_range) < self.min_distance:
            # Obstacle directly in front
            self.obstacle_detected = True
            self.obstacle_avoidance_active_pub.publish(self.obstacle_detected)
            self.stop_robot()
            rospy.loginfo("Obstaaacle detected !!!")
        else:
            rospy.loginfo("No immediate obstacle detected, you're safe.")

    def get_range(self, data, angle_start, angle_end):
        # Convert angles from degrees to radians and find indices
        start_index = index_from_angle(math.radians(angle_start), data)
        end_index = index_from_angle(math.radians(angle_end), data)

        # Check if the range wraps around the array end
        if start_index > end_index:
            # Wrap around the end of the range array
            return data.ranges[start_index:] + data.ranges[:end_index + 1]
        return data.ranges[start_index:end_index + 1]
        
    def stop_robot(self):
        # Immediately stop any motion by publishing a zero velocity command
        twist = Twist()
        twist.linear.x = 0  # Stop moving forward
        twist.angular.z = 0  # Stop any turning
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Publishing stop command.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        stop = StopAtObstacle()
        stop.run()
    except rospy.ROSInterruptException:
        pass