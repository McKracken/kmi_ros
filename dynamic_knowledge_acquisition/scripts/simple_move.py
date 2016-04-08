#!/usr/bin/env python

import roslib; roslib.load_manifest('dynamic_knowledge_acquisition')
import rospy
import actionlib
import math

#move_base_msgs
from move_base_msgs.msg import *

def simple_move():

    rospy.init_node('simple_move')

    #Simple Action Client
    sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )

    #create goal
    goal = MoveBaseGoal()

    #set goal
    goal.target_pose.pose.position.x = 1.0
    goal.target_pose.pose.position.y = 1.0
    goal.target_pose.pose.orientation.z = math.sin(math.radians(0)/2)
    goal.target_pose.pose.orientation.w = math.cos(math.radians(0)/2)

    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    #start listener
    sac.wait_for_server()

    #send goal
    sac.send_goal(goal)

    #finish
    sac.wait_for_result()

    #print result
    print sac.get_result()


if __name__ == '__main__':
    try:
        simple_move()
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
