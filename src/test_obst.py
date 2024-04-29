#!/usr/bin/env python3
#chqt
# Author : Juyeon Kim
# Date: from 2023

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

def callback(dt):
    print ('-------------------------------------------')
    print ('Range data at 0 deg:   {}'.format(dt.ranges[0]))
    print ('Range data at 15 deg:  {}'.format(dt.ranges[15]))
    print ('Range data at -15 deg:  {}'.format(dt.ranges[-15]))
    print ('Range data at 345 deg: {}'.format(dt.ranges[345]))
    print ('-------------------------------------------')
    # Find the minimum distance to any obstacle
    min_dist = np.min(dt.ranges)
    print(min_dist)
    # Set the threshold for obstacle avoidance
    thr = 0.25
     # Set distance thresholds
    min_distance = 0.2
    max_distance = 0.55
    
    if min_dist < thr:
        move.linear.x = 0.0 # stop
        move.angular.z = 0.0
        print("obstacle detected : stop")
        
         # Determine the side of the closest obstacle
        left_distances = [x for x in dt.ranges[:30]if not np.isinf(x)]
        if len(left_distances) >0:
            left_distance = min(left_distances)
            print("leftdist",left_distance)
        else:
            left_distance = max_distance
        
        right_distances = [x for x in dt.ranges[-50:]if not np.isinf(x)]
        if len(right_distances) >0:
            right_distance = min(right_distances)
            print("rightdist",right_distance)
        else:
            right_distance = max_distance
            print("rightdist=max_dist")
            
        if dt.ranges[15] > max_distance:

            move.angular.z =0.4 # rotate clockwise
            move.linear.x=0.03
        
        elif dt.ranges[-15] >max_distance:
            print("rightdistance",right_distance)
            move.angular.z = -0.4 # 
            move.linear.x=0.03
        else:
           
            move.angular.z = 0
            move.linear.x=0
    
    else :
# If there are no obstacles ahead, move forward
        move.linear.x = 0.1 # go forward (linear velocity)
        move.angular.z = 0.0 # do not rotate (angular velocity)

        
       
            
    pub.publish(move) # publish the move object


if __name__ == '__main__':
    try:
        
        move = Twist() # Creates a Twist message type object
        rospy.init_node('obstacle_avoidance_node') # Initializes a node
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  # Publisher object which will publish "Twist" type messages
                                                    # on the "/cmd_vel" Topic, "queue_size" is the size of the
                                                                # outgoing message queue used for asynchronous publishing

        sub = rospy.Subscriber("/scan", LaserScan, callback)  # Subscriber object which will listen "LaserScan" type messages
                                                            # from the "/scan" Topic and call the "callback" function
                                    # each time it reads something from the Topic

        rospy.spin() # Loops infinitely until someone stops the program execution

    except rospy.ROSInterruptException:
        pass
