#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Juyeon Kim
# Date : 2024
# Tested : works better than mybot_teleop.py


import rospy
import sys, termios, tty
import click
from std_msgs.msg import String #import
from geometry_msgs.msg import Twist #import the data type expected for t he topic cmd_vel

class MybotTeleop:
    def __init__(self, topic_name='/cmd_vel'):
        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        self.rate = rospy.Rate(3)
        self.msg = Twist()
        self.speed_linear = linear_scale
        self.speed_angular = angular_scale

    def handle_key(self, char): 
        self.msg == Twist()
        if char == 'up':
            self.msg.linear.x = self.speed_linear
            self.msg.angular.z = 0
        elif char == 'down':
            self.msg.linear.x = -self.speed_linear
            self.msg.angular.z = 0
        elif char == 'left':
            self.msg.linear.x = 0
            self.msg.angular.z = self.speed_angular
        elif char == 'right':
            self.msg.linear.x = 0
            self.msg.angular.z = -self.speed_angular
        elif char == 'stop':
            self.msg.linear.x = 0
            self.msg.angular.z = 0
        elif char == 'quit':
            rospy.signal_shutdown("Quit")   
        elif char =='slow':
            self.speed_linear -= 1
            #self.speed_angular -= 1
        elif char =='fast':
            self.speed_linear +=1
            #self.speed_angular -= 1



    def teleop_loop(self):

        # Arrow keys codes
        keys = {'\x1b[A':'up', '\x1b[B':'down', '\x1b[C':'right', '\x1b[D':'left', 's':'stop', 'q':'quit', 'a':'slow','e':'fast' }

        

        while not rospy.is_shutdown():
            mykey = click.getchar()
            if mykey in keys.keys():
                char = keys[mykey]
                self.handle_key(char)
                rospy.loginfo(f"speed_linear: {self.speed_linear}")
                rospy.loginfo(f"Linear Scale: {self.msg.linear.x }")
                rospy.loginfo(f"speed_angular: {self.speed_angular}")
                rospy.loginfo(f"angular Scale: {self.msg.linear.z }")

                self.pub.publish(self.msg)
                self.rate.sleep()
            


if __name__ == '__main__':

    try:    
        rospy.init_node('mybot_teleop')

        # Fetch parameters using private node handle
        linear_scale = rospy.get_param('~linear_scale', 1.0)  # Default value: 1.0
        angular_scale = rospy.get_param('~angular_scale', 1.0)  # Default value: 1.0
        

        rospy.loginfo(f"---------------------------")
        rospy.loginfo(f"Linear Scale: {linear_scale}")
        rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~linear_scale'), linear_scale)

        rospy.loginfo(f"Angular Scale: {angular_scale}")
        rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~angular_scale'), angular_scale)
        rospy.loginfo(f"---------------------------")
        my_bot_teleop = MybotTeleop()
        my_bot_teleop.teleop_loop()

    except rospy.ROSException as e:
        rospy.logerr(f"Error during node initialization: {str(e)}")

    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {str(e)}")
        
    #except rospy.ROSInterruptException:
    #    pass