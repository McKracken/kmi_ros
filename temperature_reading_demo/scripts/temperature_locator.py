#!/usr/bin/env python

import time
import sys
import rospy
from temperature_node.msg import *
from temperature_node.srv import *
#from turtlesim.msg import *
from tf.msg import tfMessage 
import codecs


coord_X = 0
coord_Y = 0
global writer

def position(data):
	global coord_X 
	global coord_Y
	if len(data.transforms) > 0:	
		coord_X = data.transforms[0].transform.translation.x #data.x 
		coord_Y = data.transforms[0].transform.translation.y #data.y
		#print "Position :[x=%s, y=%s]" % (str(coord_X), str(coord_Y) )
	else: print "No position detected."

def listener():
	"""
	init the node subscriber to the position
	"""
	rospy.init_node('temperature_locator', anonymous=True)
	#rospy.Subscriber('/turtle1/pose', Pose, position)
	rospy.Subscriber('/tf', Transform, position)
	


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

def temperature_reader():
	global writer
	rospy.init_node('temperature_locator', anonymous=True)
	
	rospy.wait_for_service('dht22_ask_temperature')
	
	#rospy.Subscriber('/turtle1/pose', Pose, position)
	rospy.Subscriber('/tf', tfMessage, position)
	
	while not rospy.is_shutdown():
		time.sleep(3)
		
    		try:
        		ask_temperature = rospy.ServiceProxy('dht22_ask_temperature', AskTemperature)
			response = ask_temperature('[TEMP]')
			temperature = response.reply.temperature
			humidity = response.reply.humidity
			print "Temp:%s Humidity:%s at position [%s,%s]  " % (temperature,humidity,coord_X, coord_Y)
			writer.write("%s,%s,%s,%s\n" % (temperature,humidity,coord_X,coord_Y))

		except rospy.ServiceException, e:
        		print "Service call failed: %s"%e
	sys.exit(0)

if __name__ == "__main__":
	outputFile = rospy.get_param("/temperature_locator/f")
	print outputFile
	writer = codecs.open(outputFile,"w","utf-8")
	temperature_reader()
	writer.close()
	#listener()
	#ask_temperature_client()
