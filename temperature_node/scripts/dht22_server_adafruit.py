#!/usr/bin/env python

import rospy
from temperature_node.srv import *
from temperature_node.msg import *
import sys
import socket

GLOBAL_SOCKET=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
 
def handle_ask_temperature(request): 
	print "Received request %s" % request
	GLOBAL_SOCKET.sendall(request.request) # sends request object
	sensor_response=GLOBAL_SOCKET.recv(1024)
	temperature = sensor_response.split()[0]
	humidity = sensor_response.split()[1]
	return_temperature = Temperature(float(temperature), float(humidity))
	return AskTemperatureResponse(return_temperature)

def dht22_server(host,port):
	rospy.init_node('dht22_ask_temperature_server') # declare node in ros core
	s = rospy.Service('dht22_ask_temperature', AskTemperature, handle_ask_temperature) # declare service
	print 'Ready to ask for temperature.'
	
	GLOBAL_SOCKET.connect((host,port))
	print "Connected to %s : %s" % (host,port)
	rospy.spin()


if __name__ == "__main__":
	dht22_server(rospy.get_param("/dht22_server/h"),int(rospy.get_param("/dht22_server/p")))
	#dht22_server(sys.argv[1],int(sys.argv[2]))
