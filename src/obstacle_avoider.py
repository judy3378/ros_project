#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import math

def index_from_angle(angle, data):
    # Calculate index for a given angle using the LaserScan message
    return int((angle - data.angle_min) / data.angle_increment)

class ObstacleAvoider:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('obstacle_avoider', anonymous=True)
        
        # Publishers for velocity commands and obstacle avoidance state
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.obstacle_avoidance_active_pub = rospy.Publisher('/obstacle_avoidance_active', Bool, queue_size=1)
        
        # Subscribers for LaserScan data and corridor activity status
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/corridor_active", Bool, self.corridor_callback)
        
        # Set up rate for the main loop
        self.rate = rospy.Rate(10)

        # Load parameters from the ROS parameter server
        self.use_sim = rospy.get_param('use_sim', True)

        # Obstacle detection and avoidance parameters
        self.min_distance = 0.25  # Threshold distance to detect an obstacle
        self.critical_distance = 0.17  # Safety distance to prevent collisions
        self.obstacle_detected = False  # Flag to indicate if an obstacle is detected

        # Ranges for obstacle detection
        self.front_range = []
        self.right_range = []
        self.left_range = []
        self.front_safety_range = []

        self.corridor_active = False  # Flag for corridor activity

    def corridor_callback(self, msg):
        # Update corridor active status
        self.corridor_active = msg.data

    def scan_callback(self, data):
        if not self.corridor_active:
            # Creating ranges for obstacle detection logic
            self.front_range = self.get_range(data, 335, 25)  # -25 to 25 degrees
            self.left_range = self.get_range(data, 25, 90)  # 25 to 90 degrees
            self.right_range = self.get_range(data, 270, 335)  # -90 to -25 degrees
            print("min= ", min(self.front_range))

            # Creating a safety range for obstacle avoidance logic (when an obstacle is detected)
            self.front_safety_range = self.get_range(data, 305, 55)  # -55 to 55 degrees

            # Adjust range values for the real robot (0 means no detection, use 10 instead)
            if not self.use_sim:
                range = []
                for nb in self.front_range:
                    if nb == 0:
                        nb = 10
                    range.append(nb)
                self.front_range = range

                safety_range = []
                for nb in self.front_safety_range:
                    if nb == 0:
                        nb = 10
                    safety_range.append(nb)
                self.front_safety_range = safety_range

                lrange = []
                for nb in self.left_range:
                    if nb == 0:
                        nb = 10
                    lrange.append(nb)
                self.left_range = lrange

                rrange = []
                for nb in self.right_range:
                    if nb == 0:
                        nb = 10
                    rrange.append(nb)
                self.right_range = rrange

            # Call the obstacle avoidance logic
            self.avoid_obstacle()

    def avoid_obstacle(self):
        # Obstacle avoidance logic
        if min(self.front_range) < self.min_distance:
            # Obstacle dditected in front
            self.obstacle_detected = True
            self.obstacle_avoidance_active_pub.publish(self.obstacle_detected)
            rospy.logwarn("Obstaaacle detected !!!")
        else:
            rospy.loginfo("No immediate obstacle detected, you're safe.")

        # React based on obstacle detection
        if self.obstacle_detected :
            if min(self.front_safety_range) < self.critical_distance:
                # Too close to an obstacle, move backward
                self.move_backward()
                rospy.loginfo("Too close! Moving backward...")

            elif min(self.front_safety_range) < self.min_distance:
                # Obstacle detected, decide to turn left or right based on available space
                    if min(self.left_range) < min(self.right_range):
                        # More space on the right side
                        self.turn_right()
                        rospy.loginfo("Turning right...")

                    else :
                        # More space on the left side
                        self.turn_left()
                        rospy.loginfo("Turning left...")
            
            elif  min(self.front_safety_range) > self.min_distance and ((min(self.left_range) < self.critical_distance or min(self.right_range) < self.critical_distance)) :
                    # Move forward if the path ahead is clear and sides are critically close
                    self.move_forward()
                    rospy.logwarn("Moving forward...")

            else : 
                # No obstacle detected, deactivate obstacle avoidance
                self.obstacle_detected = False
                self.obstacle_avoidance_active_pub.publish(self.obstacle_detected)
                rospy.loginfo("No more obstacle!")

    def get_range(self, data, angle_start, angle_end):
        # Convert angles from degrees to radians and find indices
        start_index = index_from_angle(math.radians(angle_start), data)
        end_index = index_from_angle(math.radians(angle_end), data)

        # Check if the range wraps around the array end
        if start_index > end_index:
            # Wrap around the end of the range array
            return data.ranges[start_index:] + data.ranges[:end_index + 1]
        return data.ranges[start_index:end_index + 1]

    def move_forward(self):
        # Publish a command to move forward
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)

    def move_backward(self):
        # Publish a command to move backward
        twist = Twist()
        twist.linear.x = -0.1
        self.cmd_vel_pub.publish(twist)

    def turn_right(self):
        # Publish a command to turn right
        twist = Twist()
        twist.angular.z = -0.5
        twist.linear.x = 0.1  # Slowly moving forward while turning
        self.cmd_vel_pub.publish(twist)

    def turn_left(self):
        # Publish a command to turn left
        twist = Twist()
        twist.angular.z = 0.5
        twist.linear.x = 0.1  # Slowly moving forward while turning
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        # Immediately stop any motion by publishing a zero velocity command
        twist = Twist()
        twist.linear.x = 0  # Stop moving forward
        twist.angular.z = 0  # Stop any turning
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Publishing stop command.")

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        # Create an instance of the ObstacleAvoider and run it
        obstacle_avoider = ObstacleAvoider()
        obstacle_avoider.run()
    except rospy.ROSInterruptException:
        pass