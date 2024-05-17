#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import click

from geometry_msgs.msg import Twist #import the expected data type of the topic
from std_msgs.msg import Bool

# Arrow keys codes
keys = {'\x1b[A':'up', '\x1b[B':'down', '\x1b[C':'right', '\x1b[D':'left', 's':'stop', 'q':'quit'}

# Initialize the global variable
obstacle_avoidance_active = False

def obstacle_avoidance_callback(msg):
    global obstacle_avoidance_active
    obstacle_avoidance_active = msg.data

def stop(msg):
    msg.linear.x = 0.0
    msg.angular.z = 0.0

if __name__ == '__main__':

    try:
        rospy.init_node('mybot_teleop', anonymous=True)

        # Load the velocity topic name from ROS parameter server, defaulting to /cmd_vel
        velocity_topic = rospy.get_param("velocity_topic", "/cmd_vel")

        # Define a publisher on the dynamically defined velocity topic with the imported data type
        pub = rospy.Publisher(velocity_topic, Twist, queue_size=10) 
        # pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10) # For turtlesim
        # rate = rospy.Rate(10)  # 10Hz This command isn't necessary in that case because the rate is determined by the action clicking a key

        # Define subscriber
        rospy.Subscriber("/obstacle_avoidance_active", Bool, obstacle_avoidance_callback)

        msg = Twist()
        
        while not rospy.is_shutdown():
            # Load scale parameters from the parameter server, defaulting to 1.0 if not specified
            linear_scale = rospy.get_param('linear_scale', 0.5) # Default value for turtlesim = 1.0
            angular_scale = rospy.get_param('angular_scale', 0.5) # Default value for turtlebot = 0.5
        
            if not obstacle_avoidance_active:
                mykey = click.getchar()
                char = ''
                if mykey in keys.keys():
                    char=keys[mykey]

                if char == 'up':    # UP key
                    msg.linear.x = linear_scale
                    msg.angular.z = 0.0
                
                if char == 'down':  # DOWN key
                    msg.linear.x = -linear_scale
                    msg.angular.z = 0.0
                
                if char == 'right':  # RIGHT key
                    msg.linear.x = 0.0
                    msg.angular.z = -angular_scale
                    
                if char == 'left': # LEFT key
                    msg.linear.x = 0.0
                    msg.angular.z = angular_scale
            
                if char == "quit":  # QUIT
                    break
            else :
                stop(msg)
            
            pub.publish(msg)

    except rospy.ROSInterruptException:
        pass