#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""This code functions once the robot is in the beginnning of the corridor on the red line"""

import rospy
from geometry_msgs.msg import Twist #sends velocity command to the cmd/vel topic
from sensor_msgs.msg import LaserScan #receives LIDAR scan from the topic scan
from projet.msg import LinePosition
from projet.srv import StartCorridor
from std_msgs.msg import Bool
import numpy as np
import time

class CorridorNavigator:
    def __init__(self):
        # Initialize ROS node and subscribers
        rospy.init_node('Corridor_Gazebo')
        rospy.Subscriber("/line_position", LinePosition, self.line_callback)
        rospy.Subscriber("/obstacle_avoidance_active", Bool, self.obstacle_avoidance_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.corridor_active_pub = rospy.Publisher('/corridor_active', Bool, queue_size=1)

        # Set up parameters
        self.use_sim = rospy.get_param('use_sim', True)

        # Set movement speed
        self.linear_speed = 0.2
        self.angular_speed = 0.3

        # Set distance thresholds
        self.min_distance = 0.2
        self.max_distance = 0.55

        # Set initial state
        self.state = 'follow_left_wall'
        print("init")

        self.corridor_active = False
        self.obstacle_avoidance_active = False
        self.is_yellow_detected = False
        self.is_white_detected = False

        # Keep track of mission completion
        self.completed_mission = False

        # Initialize publisher and start moving robot
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Detector readiness flag
        self.detector_ready = False

        # Start processing the corridor navigation
        self.start_processing()

    def start_processing(self):
        rospy.wait_for_service('start_corridor')
        timeout = time.time() + 10.0  # 10 seconds from now
        while time.time() < timeout:
            try:
                start_corridor = rospy.ServiceProxy('start_corridor', StartCorridor)
                response = start_corridor()
                if response.started:
                    rospy.logwarn("Starting corridor processing: %s", response.message)
                    self.detector_ready = True
                    self.run()
                    return
                else:
                    rospy.logwarn("Line detector not ready: %s", response.message)
                    rospy.sleep(1)  # wait for a second before retrying
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)
                rospy.sleep(1)  # wait for a second before retrying

        rospy.logerr("Failed to start corridor processing after multiple attempts.")

    def line_callback(self, msg):
        # Update line detection status based on received message
        if self.use_sim: # In the simulation
            self.is_yellow_detected = msg.yellow_detected
            self.is_white_detected = msg.white_detected
        else: # In the real world
            self.is_yellow_detected = msg.green_detected
            self.is_white_detected = msg.red_detected
        self.last_received = rospy.get_time()

    def obstacle_avoidance_callback(self, msg):
        # Update obstacle avoidance status
        self.obstacle_avoidance_active = msg.data

    def scan_callback(self, scan_msg):
        # Process LIDAR scan data if the detector is ready
        if self.detector_ready:
            if (not self.is_yellow_detected and not self.is_white_detected and not self.obstacle_avoidance_active) or self.corridor_active:
                self.corridor_active = True
                self.corridor_active_pub.publish(self.corridor_active)
                rospy.logwarn("Following corridor !")

                # Process distances from LIDAR scan
                left_distances = [x for x in scan_msg.ranges[:50] if not np.isinf(x)]
                right_distances = [x for x in scan_msg.ranges[-50:] if not np.isinf(x)]

                # Calculate minimum distances or set to max_distance if no valid readings
                left_distance = min(left_distances) if left_distances else self.max_distance
                right_distance = min(right_distances) if right_distances else self.max_distance

                # Call the corridor navigation handler
                self.handle_corridor_navigation(left_distance, right_distance)

    def handle_corridor_navigation(self, left_distance, right_distance):
        # Navigate the corridor based on LIDAR data
        if self.state == 'follow_left_wall':
            if left_distance > self.max_distance:
                # Turn left to get closer to left wall
                self.move(0, self.angular_speed)
            elif left_distance < self.min_distance:
                # Turn right to avoid collision with left wall
                self.move(0, -self.angular_speed)
            else:
                # Move forward along left wall
                self.move(self.linear_speed, 0)
                if right_distance < self.max_distance:
                    self.state = 'follow_right_wall'

        elif self.state == 'follow_right_wall':
            if right_distance > self.max_distance:
                # Turn right to get closer to right wall
                self.move(0, -self.angular_speed)
            elif right_distance < self.min_distance:
                # Turn left to avoid collision with right wall
                self.move(0, self.angular_speed)
            else:
                # Move forward along right wall
                self.move(self.linear_speed, 0)
                if left_distance < self.max_distance:
                    self.state = 'follow_left_wall'
        
        # Evaluate whether the corridor navigation should stop
        if left_distance >= self.max_distance and right_distance >= self.max_distance:
            self.complete_mission()

    def complete_mission(self):
        # Logic to handle the end of the mission
        self.completed_mission = True
        self.corridor_active = False
        self.corridor_active_pub.publish(self.corridor_active)
        rospy.logwarn("Out of the corridor !")
        self.move(0, 0)
        rospy.signal_shutdown("Robot has reached the end of the corridor")


    def move(self, linear, angular):
        # Move robot using Twist message
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        # Initialize node and start moving robot
        robot = CorridorNavigator()

        # Move robot forward until it reaches the end of the corridor
        while not rospy.is_shutdown()and not robot.completed_mission:
            robot.move(robot.linear_speed, 0)

    except rospy.ROSInterruptException:
        pass