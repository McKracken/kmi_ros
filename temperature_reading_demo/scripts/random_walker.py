#!/usr/bin/env python

import random
import rospy
import sys
from geometry_msgs.msg import Twist, Quaternion
from tf.msg import tfMessage

class random_walker():
    
    def __init__(self):
        """ 
        constructor 
        """
        
        # register this function to be called on shutdown
        rospy.on_shutdown(self.shutdown)
        
        # change roomba to full mode
        rospy.Publisher('mode', String)
        
        # give the publisher a bit of time to connect
        rospy.sleep(1)
        
        self.twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        
    def shutdown():    
        # stop the robot
        twist = Twist()
        self.pub.publish(twist)
        

if __name__=="__main__":
    rospy.init_node('walker')
    random_walker()