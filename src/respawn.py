#!/usr/bin/env python3
# Author: Juyeon Kim
# Date : 2024
# Tested :

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

def respawn_robot():
    # Create a ServiceProxy to call the Gazebo respawn_robot service
    respawn = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    
    # Call the service to respawn the robot
    try:
        response = respawn()
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        return False
    
def talk():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('keyboard_control', anonymous=True)

    while not rospy.is_shutdown():
        message = Twist()
        key_pressed = input("Press 'r' to respawn the robot: ")

        if key_pressed == 'r':
            rospy.loginfo("Respawning robot...")
            respawn_robot()
            rospy.sleep(0.1)  # Wait for robot to reset
            rospy.loginfo("Robot respawned successfully!")
            message.linear.x = 0.0
            message.angular.z = 0.0
            pub.publish(message)

    rospy.loginfo("Shutting down...")

if __name__ == '__main__':
    try:
        talk()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupted.")  # Handle interrupt gracefully