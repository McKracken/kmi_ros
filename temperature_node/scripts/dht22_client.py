#!/usr/bin/env python

import sys
import rospy
from temperature_node.msg import *
from temperature_node.srv import *

def ask_temperature_client():
    rospy.wait_for_service('dht22_ask_temperature')
    try:
        ask_temperature = rospy.ServiceProxy('dht22_ask_temperature', AskTemperature)
        response = ask_temperature('[TEMP]')
        temperature = response.reply.temperature
	humidity = response.reply.humidity
	print "Temp:%s Humidity:%s" % (temperature,humidity)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
	ask_temperature_client()
