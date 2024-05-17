#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from projet.msg import LinePosition
from std_msgs.msg import Bool

class LineFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('line_follower', anonymous=True)

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/line_position", LinePosition, self.line_callback)
        rospy.Subscriber("/obstacle_avoidance_active", Bool, self.obstacle_avoidance_callback)
        rospy.Subscriber("/corridor_active", Bool, self.corridor_callback)

        # Rate of processing loop
        self.rate = rospy.Rate(10)  # 10 Hz

        # Time tracking for message reception
        self.last_received = rospy.get_time()

        # Line detection variables
        self.is_yellow_detected = False
        self.is_white_detected = False
        self.yellow_x = 0
        self.white_x = 0

        # PARAMETERS
        self.use_sim = rospy.get_param('use_sim', True)  # Whether to use simulation settings
        # Velocity limits
        self.max_linear_velocity = rospy.get_param('max_linear_velocity', 0.27)
        self.min_linear_velocity = rospy.get_param('min_linear_velocity', 0.08)
        self.angular_velocity_factor = rospy.get_param('angular_velocity_factor', 0.065)

        # State flags
        self.obstacle_avoidance_active = False
        self.corridor_active = False

    def line_callback(self, msg):
        # Update line detection based on the received message
        if self.use_sim: # In the simulation
            self.is_yellow_detected = msg.yellow_detected
            self.is_white_detected = msg.white_detected
            self.yellow_x = msg.yellow_x
            self.white_x = msg.white_x
        else: # In the real world
            self.is_yellow_detected = msg.green_detected
            self.is_white_detected = msg.red_detected
            self.yellow_x = msg.green_x
            self.white_x = msg.red_x
        self.last_received = rospy.get_time()

    def obstacle_avoidance_callback(self, msg):
        # Update obstacle avoidance state
        self.obstacle_avoidance_active = msg.data

    def corridor_callback(self, msg):
        # Update corridor mode state
        self.corridor_active = msg.data

    def follow_line(self):
        # Follow the line if no obstacle is detected or if in corridor mode not active
        if not self.obstacle_avoidance_active or self.corridor_active:
            # Set image width based on simulation or real robot
            image_width = 320 if self.use_sim else 640

            # Calculate the midpoint of the detected lines
            midpoint = (self.yellow_x + self.white_x) / 2
            image_center_x = image_width // 2
            error = midpoint - image_center_x

            # Normalizing the error to a range of [-1, 1] for velocity adjustment
            normalized_error = abs(error) / (image_width / 2)

            # Normalizing the error to a range of [-1, 1] for velocity adjustment
            normalized_error = abs(error) / (image_width / 2)

            # Dynamic linear velocity adjustment
            linear_velocity = self.max_linear_velocity * (1 - normalized_error)
            linear_velocity = max(linear_velocity, self.min_linear_velocity)  # Ensure velocity does not go below minimum

            # Proportional control for angular velocity
            angular_velocity = -error * self.angular_velocity_factor

            # Create and publish Twist message
            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist)
        else:
            rospy.loginfo("Obstacle detected!")

    def search_for_line(self):
        # Implement behavior to search for the line
        twist = Twist()
        if not self.is_yellow_detected and not self.is_white_detected:
            # If both lines are lost, rotate slowly to search
            twist.angular.z = 0.3
        elif not self.is_yellow_detected:
            # Turn in place to find yellow line
            rospy.logwarn("Yellow line lost. Searching ...")
            twist.angular.z = 0.5
        elif not self.is_white_detected:
            # Turn in place to find white line
            rospy.logwarn("White line lost. Searching...")
            twist.angular.z = -0.5
        self.cmd_vel_pub.publish(twist)

    def run(self):
        # Main loop of the line follower logic
        while not rospy.is_shutdown():
            # Check if the line data is recent
            if rospy.get_time() - self.last_received < 1.0:  # 1 second timeout
                if not self.obstacle_avoidance_active:
                    if self.is_yellow_detected and self.is_white_detected:
                        # Check if lines are detected correctly
                        if self.yellow_x > self.white_x:  # Yellow should be on the left
                            self.is_white_detected = False  # Invalidate this detection and search again
                            rospy.logwarn("Yellow line detected on the wrong side. Adjusting...")
                            self.search_for_line()
                        else :
                            self.follow_line()
                    else:
                        self.search_for_line()
            self.rate.sleep()

if __name__ == '__main__':
    controller = LineFollower()
    controller.run()