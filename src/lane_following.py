#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author : Juyeon Kim
# Date: from 2023

## add a function to set the cmd vel to zero when closing the node

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


def nouvelle_zone(img,):
    """ Function to redefine the image from the camera by creating a rectangular mask.
    Covers the two-thirds of the input image to avoid detecting objects higher than the lanes.
    Returns the masked image"""
    height, width, _ = img.shape
    rectangle= np.array([
                        [(0, height), (0, 2*height//3), (width, 2*height//3), (width, height)]
                        ])
    mask = np.zeros_like(img)
    mask = cv2.fillPoly(mask, rectangle, [255, 255, 255])
    mask = cv2.bitwise_and(img, mask)
    return mask

def read_image_callback(msg):
    """Callback function to process the received image
    """
    real_situation =0 # on simuation

    if real_situation == 1: # on real robot
        biais = 0
        error_line = 100
    else :
        biais = 15
        error_line = 100
        
    # convert the received image from camera to OpenCV
    bridge = CvBridge()
    cvImage = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    height, width,_ =cvImage.shape

    cvImage = nouvelle_zone(cvImage)
    
    # create a copy of the image for further processing
    copy= cvImage
    
    #  convert the iamge into the HSV color space.
    hsv = cv2.cvtColor(cvImage, cv2.COLOR_BGR2HSV)
    hsv_red=cv2.cvtColor(copy,cv2.COLOR_BGR2HSV)

    # find the upper and lower bounds of the yellow color (line on the left)
    yellowLower = (20,100,100)
    yellowUpper = (37,255,255)
    
    # define a mask using the lower and upper bounds of the yellow color
    mask_yellow = cv2.inRange(hsv, yellowLower, yellowUpper)

    # find the upper and lower bounds of the white color (line on the right)
    whiteLower = (0, 0, 200)
    whiteUpper = (179, 30, 255)
    
    # define a mask using the lower and upper bounds of the white color
    mask_white = cv2.inRange(hsv, whiteLower, whiteUpper)

    # define a mask using the lower and upper bounds of the two types of red color for real robot use
    red_down_low=(0,100,20)
    red_down_high=(20,255,255)

    red_up_low=(160,100,20)
    red_up_high=(179,255,255)
    
    # define a polygon representing the region of interest and draw on the copy image
    pts = np.array([[0, height], [width//3, height//3], [2*width//3, height//3], [width, height]])
    cv2.polylines(copy, [pts], True, (255, 0, 0), thickness=2)
    
    # convert the image to grayscale 
    gray = cv2.cvtColor(copy ,cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)
    
    # define a red mask to isolate red line
    redLower = (0, 0, 120)
    redUpper = (70, 70, 255)
    red_mask = cv2.inRange(copy, redLower, redUpper)
    red_mask = cv2.bitwise_and(red_mask, thresh)
    
    # detect contours in the red mask
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # draw the contours on the copy image
    cv2.drawContours(copy, contours, -1, (0, 255, 0), 2)
 
    # combine the white and yellow masks
    mask_white_yellow= cv2.bitwise_or(mask_white, mask_yellow)
    
    # apply the combined mask to the original image
    result = cv2.bitwise_and(cvImage, cvImage, mask=mask_white_yellow)
    
    # calculate the moments for the white, yellow, and red mask to determine their centers
    Mw=cv2.moments(mask_white)
    My= cv2.moments(mask_yellow)
    Mr= cv2.moments(red_mask)

    red_line=False
    # different scenarios based on the detected lines
    # The detected positions are printed, and circles are drawn on the result image.
    
    # If both yellow and white lines are detected, the center position between them is calculated
    if My["m00"] > 0 and Mw["m00"] > 0 :
        cX = (int(Mw["m10"] / Mw["m00"]) + int(My["m10"] / My["m00"]))//2 + biais
        cY = (int(Mw["m01"] / Mw["m00"]) + int(My["m01"] / My["m00"]))//2

        cv2.circle(result,(cX,cY),10,(0,0,255),5,)
        print("les deux lignes:",(cX,cY))

    # If only the left line is detected, an offset is added to the calculated position
    elif My["m00"] > 0  : 
        print("on utilise erroer line")
        cX = int(My["m10"] / My["m00"]) + error_line
        cY = int(My["m01"] / My["m00"])

        cv2.circle(result,(cX, cY),10,(0,0,255),3,)
        print("ligne de gauche:",(cX,cY))

    # Same is done for the right line
    elif Mw["m00"] >0 : 
        print("on utilise error line")
        cX = int(Mw["m10"] / Mw["m00"]) - error_line
        cY = int(Mw["m01"] / Mw["m00"])

        cv2.circle(result,(cX, cY),10,(0,0,255),3,)
        print("ligne de droite",(cX,cY))
    
    #  If a red line is detected, 
    elif Mr["m00"]>300000:
        red_line=True
        print(red_line)
    
   
    """This section creates a Twist message for controlling the robot's movement. 
    The twist object is modified based on the calculated position."""
    
    twist = Twist()

    #  If the position is to the right or left of the desired range, the robot turns accordingly.
    if cX > 55*width//100 :
        print(">54, Turn right") 
        twist.angular.z = -0.01*np.abs(cX - 54*width//100)
        twist.linear.x = 0.04
    
    elif cX < 45*width//100 :
        print("<46, Turn left ")
        twist.angular.z = 0.01*np.abs(cX - 46*width//100)
        twist.linear.x = 0.04
    
    # If the position is within the desired range, the robot moves forward    
    else:
        print("In the middle of the lane")
        twist.linear.x = 0.4
        twist.angular.z = 0
    
    # The twist message is published to the cmd_vel topic    
    pub.publish(twist)
         
    # display the result image
    #cv2.imshow("camera_output.jpeg", mask_red)    
    cv2.imshow("camera_output.jpeg", result)
    
    cv2.waitKey(1)

if __name__ == '__main__':
    try:
        # initialize the ROS node
        rospy.init_node('lane_following_test_node', anonymous=False)
       
        # create a subscriber for the camera/image topic
        sub = rospy.Subscriber('camera/image', Image, read_image_callback )
        
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        rospy.spin()
        
            
    except rospy.ROSInterruptException:
        pass
   
