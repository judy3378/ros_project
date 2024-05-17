#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from projet.msg import LinePosition
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from projet.srv import StartCorridor, StartCorridorResponse


class LineDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('line_detector', anonymous=True)
        
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        
        # Load parameters from the ROS parameter server
        self.load_parameters()
        
        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber(self.image_topic, self.topic_type, self.callback, queue_size=1)
        
        # Publisher for line positions
        self.line_pub = rospy.Publisher("/line_position", LinePosition, queue_size=10)
        
        # Subscriber for corridor activity status
        rospy.Subscriber("/corridor_active", Bool, self.corridor_callback)
        
        # Delay to ensure the publisher is fully ready
        rospy.sleep(1)

    def load_parameters(self):
        # Read parameters from the ROS parameter server
        self.use_sim = rospy.get_param('use_sim', True)
        self.image_topic = rospy.get_param('image_topic', '/camera/image' if self.use_sim else '/cam/image/compressed')#'/raspicam_node/image/compressed')
        self.topic_type = CompressedImage if not self.use_sim else Image

        # Default color ranges for simulation and real environments
        default_color_ranges = {
            'simulation': {
                'yellow': ([20, 100, 100], [30, 255, 255]),
                'white': ([0, 0, 200], [180, 20, 255])
            },
            'real': {
                'green': ([70, 42, 70], [102, 150, 175]),
                'red_lower': ([0, 70, 50], [10, 255, 255]),
                'red_upper': ([160, 60, 80], [180, 255, 255])  
            }
        }

        self.color_ranges = rospy.get_param('color_ranges', default_color_ranges)
        rospy.loginfo(f"Loaded color_ranges: {self.color_ranges}")
        rospy.loginfo(f"Type of color_ranges: {type(self.color_ranges)}")

        # Initial states
        self.corridor_active = False
        self.detector_ready = False  # Flag to indicate readiness
        
        # Check in which environment we are (simulation or real world)
        self.env = 'simulation' if self.use_sim else 'real'
        print("I'm in ", self.env, " environment.")

    def callback(self, data):
        # Convert the image from ROS to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") if self.use_sim else self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Process the received image
        self.process_image(cv_image)

    def corridor_callback(self, msg):
        # Update corridor active status
        self.corridor_active = msg.data

    def process_image(self, cv_image):
        if not self.corridor_active:
            # Define the ROI (Region of interest)
            height, width, _ = cv_image.shape
            roi_height = height * 2 // 5 if self.use_sim else height // 2
            roi = cv_image[(height - roi_height):height, 0:width]

            # Convert ROI to HSV color space and apply Gaussian Blur
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            hsv = cv2.GaussianBlur(hsv, (5, 5), 0)

            
            masks = {}
            combined_mask = None

            # Create masks for each color range
            for color, (lower, upper) in self.color_ranges[self.env].items():
                lower_np = np.array(lower, dtype="uint8")
                upper_np = np.array(upper, dtype="uint8")

                # Threshold the HSV image to get only the colors in defined range
                mask = cv2.inRange(hsv, lower_np, upper_np)

                # Combine red_lower and red_upper into a single red mask
                if 'red' in color:  # Applies to 'red_lower' and 'red_upper'
                    if 'red' in masks:
                        masks['red'] = cv2.bitwise_or(masks['red'], mask)
                    else:
                        masks['red'] = mask
                else:
                    masks[color] = mask

                if combined_mask is None:
                    combined_mask = mask
                else:
                    combined_mask = cv2.bitwise_or(combined_mask, mask)

            if combined_mask is not None:
                # Create a combined image with detected lines
                combined_image = cv2.bitwise_and(roi, roi, mask=combined_mask)
                self.detect_lines(masks, combined_image)
                cv2.imshow("Detected Lines", combined_image)
                cv2.waitKey(3)
                

    def detect_lines(self, masks, combined_image):
        # Detect line positions, draw circles, and publish messages
        line_position_msg = LinePosition()
        for color, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            detected = False
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(combined_image, (cX, cY), 10, (0, 255, 255), -1)
                    setattr(line_position_msg, f'{color}_x', cX)
                    setattr(line_position_msg, f'{color}_y', cY)
                    setattr(line_position_msg, f'{color}_detected', True)
                    detected = True
            if not detected:
                setattr(line_position_msg, f'{color}_detected', False)

        if self.line_pub.get_num_connections() > 0:
            self.line_pub.publish(line_position_msg)
            self.detector_ready = True  # Set to True once lines are detected

    
    def start_corridor_service(self, request):
        # Service to start corridor mode if line detection is ready
        if self.detector_ready:
            return StartCorridorResponse(started=True, message="Line detection ready")
        else:
            return StartCorridorResponse(started=False, message="Line detection not ready yet")

def main():
    # Main function to initialize the node and start the service
    ld = LineDetector()
    service = rospy.Service('start_corridor', StartCorridor, ld.start_corridor_service)
    rospy.spin()

if __name__ == '__main__':
    main()
