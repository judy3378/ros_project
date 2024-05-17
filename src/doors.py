#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Juyeon Kim
# Date: 2024

import rospy
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan, CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import math

def index_from_angle(angle, data):
    # Calculate index for a given angle using the LaserScan message
    return int((angle - data.angle_min) / data.angle_increment)

class Door:
    
    def __init__(self):
        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        
        # Load parameters from the ROS parameter server
        self.load_parameters()
    
        self.image_sub = rospy.Subscriber(self.image_topic, self.topic_type, self.image_callback, queue_size=10)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        self.green_detected = False
        self.blue_detected = False
        self.left_range = 0.0
        self.right_range = 0.0
        self.range_threshold = 0.15
        self.min_distance = 0.2  # Minimum distance to consider for obstacles
        self.critical_distance = 0.12  # Critical distance for close obstacles
        self.obstacle_detected = False
        
        # Flag to determine which color object to detect
        self.detecting_blue = True
    
    def load_parameters(self):
        # Read parameters from the ROS parameter server
        self.use_sim = rospy.get_param('use_sim', True)
        self.image_topic = rospy.get_param('image_topic', '/camera/image' if self.use_sim else '/cam/image/compressed')
        self.topic_type = CompressedImage if not self.use_sim else Image

        # Green color thresholds
        self.green_H_l = rospy.get_param("/green_H_l", 27)
        self.green_S_l = rospy.get_param("/green_S_l", 161)
        self.green_V_l = rospy.get_param("/green_V_l", 117)
        self.green_H_u = rospy.get_param("/green_H_u", 86)
        self.green_S_u = rospy.get_param("/green_S_u", 255)
        self.green_V_u = rospy.get_param("/green_V_u", 255)
        
        # Blue color thresholds
        self.blue_H_l = rospy.get_param("/blue_H_l", 110)
        self.blue_S_l = rospy.get_param("/blue_S_l", 255)
        self.blue_V_l = rospy.get_param("/blue_V_l", 59)
        self.blue_H_u = rospy.get_param("/blue_H_u", 130)
        self.blue_S_u = rospy.get_param("/blue_S_u", 255)
        self.blue_V_u = rospy.get_param("/blue_V_u", 255)
        
        # Check in which environment we are (simulation or real world)
        env = 'simulation' if self.use_sim else 'real'
        # print("I'm in ", env, " environment.")
        self.detecting_blue = True
        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        hsv, mask = self.process_image(cv_image)
        self.update(mask, hsv)
        cv2.imshow('cv image', cv_image)
        cv2.waitKey(1)
                            
    def process_image(self, image):
        
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        if self.detecting_blue:
            lower_limit = np.array([self.blue_H_l, self.blue_S_l, self.blue_V_l])
            upper_limit = np.array([self.blue_H_u, self.blue_S_u, self.blue_V_u])
        else:
            lower_limit = np.array([self.green_H_l, self.green_S_l, self.green_V_l])
            upper_limit = np.array([self.green_H_u, self.green_S_u, self.green_V_u])
        
        mask = cv2.inRange(hsv_image, lower_limit, upper_limit)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(hsv_image, contours, -1, (0, 255, 0), 3)
        
        # for contour in contours:
        #     print(f"contour {contour} area {cv2.contourArea(contour)}")

        large_contours = [contour for contour in contours if cv2.contourArea(contour) >= 100]  # Filter out small contours
        filtered_contours = []


        for contour in large_contours:
            
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
            if len(approx) >= 4:  # Filtering for rectangular or cylindrical shapes
                filtered_contours.append(contour)

        centroids = []
        for contour in filtered_contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centroids.append((cX, cY))
                cv2.circle(hsv_image, (cX, cY), 5, (0, 0, 255), -1)
                
            x,y,w,h = cv2.boundingRect(contour)
            aspect_ratio = float(w) /h
            if aspect_ratio < 1.5:
                center_x = x+w /2
                center_y = y+h/2
                
                cv2.rectangle(image, (x,y), (x + w, y+h), (0, 255, 0),2)

        if centroids:
            if len(centroids) >= 2:
                # Compute middle point between two centroids
                mid_x = int((centroids[0][0] + centroids[1][0]) / 2)
                mid_y = int((centroids[0][1] + centroids[1][1]) / 2)
                cv2.circle(hsv_image, (mid_x, mid_y), 5, (255, 0, 0), -1)
                self.target_x = mid_x
                self.target_y = mid_y
            else:
                self.target_x, self.target_y = centroids[0]
            self.green_detected = True if not self.detecting_blue else False
            self.blue_detected = True if self.detecting_blue else False
        else:
            self.green_detected = False
            self.blue_detected = False

        cv2.imshow('process image', mask)
        cv2.waitKey(1)
        
        return hsv_image, mask
    def get_range(self, data, angle_start, angle_end):
        start_index = index_from_angle(math.radians(angle_start), data)
        end_index = index_from_angle(math.radians(angle_end), data)

        if start_index > end_index:
            return data.ranges[start_index:] + data.ranges[:end_index + 1]
        return data.ranges[start_index:end_index + 1]
    
    def laser_callback(self, data):
        self.front_range = self.get_range(data, 340, 20)
        filtered_front = list(filter(lambda x: x != 0.0, self.front_range))
        
        self.left_range = self.get_range(data, 25, 90)
        self.right_range = self.get_range(data, 270, 335)
        self.front_safety_range = self.get_range(data, 310, 50)
        self.filtered_front_safety = list(filter(lambda x: x != 0.0, self.front_safety_range))

        if min(self.front_range) < self.min_distance:
            self.obstacle_detected = True
            # print("Obstacle detected in the front!", min(self.front_range))
        elif min(self.left_range) < self.min_distance:
            self.obstacle_detected = True
            # print("Obstacle detected in the left!", min(self.left_range))
        elif min(self.right_range) < self.min_distance:
            self.obstacle_detected = True
            # print("Obstacle detected in the right!", min(self.right_range))
        else:
            self.obstacle_detected = False
    
    def avoid_obstacle(self):
        # React based on obstacle detection
        if self.obstacle_detected:
            if min(self.filtered_front_safety) < self.critical_distance:
                self.move_backward()
                # rospy.loginfo("Moving backward...")
                time.sleep(2)
                # print("Stopping")
                self.obstacle_detected = False
            elif min(self.filtered_front_safety) < self.min_distance:
                if min(self.left_range) < min(self.right_range):
                    # More space on the right side to keep going
                    self.turn_right()
                    # rospy.loginfo("Turning right...")
                else:
                    # More space on the left side to keep going
                    self.turn_left()
                    # rospy.loginfo("Turning left...")
            elif min(self.filtered_front_safety) > self.min_distance and (min(self.left_range) < self.min_distance or min(self.right_range) < self.min_distance):
                self.move_forward()
                # rospy.loginfo("Moving forward...")
            else:
                self.obstacle_detected = False
                # rospy.loginfo("No more obstacle")
    
    def update(self, mask, image):
        if self.blue_detected or self.green_detected:
            self.move_robot(image.shape[1], image.shape[0], self.target_x, self.target_y)
                
            if self.obstacle_detected:
                if min(self.filtered_front_safety) < self.critical_distance:
                    # Move backward and turn left or right depending on the obstacle position
                    if min(self.left_range) < min(self.right_range):
                        # More space on the right side to keep going
                        self.move_backward_left()
                        # rospy.loginfo("Moving backward and turning left to avoid obstacle...")
                    else:
                        # More space on the left side to keep going
                        self.move_backward_right()
                        # rospy.loginfo("Moving backward and turning right to avoid obstacle...")
                    time.sleep(2)  # Adjust sleep duration as needed
                    # print("Stopping")
                    self.obstacle_detected = False
                    
                elif min(self.filtered_front_safety) < self.critical_distance:
                    # Obstacle detected, decide to turn left or right based on available space
                    if min(self.left_range) < min(self.right_range):
                        # More space on the right side
                        self.turn_right()
                        # rospy.loginfo("Turning right...")

                    else:
                        # More space on the left side
                        self.turn_left()
                        # rospy.loginfo("Turning left...")
        
                else:
                    # No immediate obstacle detected
                    self.obstacle_detected = False
                    rospy.loginfo("No more obstacle")
                    

        else:
            # Stop the robot if no object is detected
            self.stop_robot()
            # rospy.loginfo("Searching for object...")
        
        # Check if large contours of blue are detected
        large_blue_contours = [contour for contour in cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0] if cv2.contourArea(contour) >=100]
        if not large_blue_contours or self.target_x <= 50 or self.target_x >=image.shape[1] - 50:
            # No large blue contours or robot is at the edges of the screen
            self.blue_detected = False
            self.move_forward()
        

        # print("Detecting blue:", self.detecting_blue)
        # print("Blue detected:", self.blue_detected)

        # Transition from detecting blue to green
        if self.detecting_blue and not self.blue_detected:
            self.detecting_blue = False
            
            # rospy.loginfo("Switching to green object detection...")

        cv2.imshow('update image', image)
        cv2.waitKey(1)

    def move_backward_left(self):
        self.publish_velocity(-0.11, 0.3)

    def move_backward_right(self):
        self.publish_velocity(-0.11, -0.3)
        
    def move_robot(self, image_width, image_height, center_x, center_y):
        if self.green_detected or self.blue_detected:
            error_x = image_width / 2 - center_x
            linear_speed = 0.05
            angular_speed = 0.01 * error_x
            self.publish_velocity(linear_speed, angular_speed)
        
    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)
        # rospy.loginfo("Shutting down. Robot stopped.")
    
    def move_backward(self):
        self.publish_velocity(-0.1, 0.0)
    
    def turn_right(self):
        self.publish_velocity(0.0, -1.0)
    
    def turn_left(self):
        self.publish_velocity(0.0, 1.0)
    
    def move_forward(self):
        self.publish_velocity(0.3, 0.0)
        
    def publish_velocity(self, linear_speed, angular_speed):
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist_msg)

def main():
    rospy.init_node('Door', anonymous=True)
    door = Door()
    door.load_parameters()
    rate = rospy.Rate(10)
    
    # Reset parameters and flags
    door.detecting_blue = True
    door.green_detected = False
    door.blue_detected = True
    door.obstacle_detected = False
    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == '__main__':
    main()
