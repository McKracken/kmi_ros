#!/usr/bin/env python

import rospy
import time
import json
import math
import os
import codecs
from move_base_msgs.msg import *
import actionlib
from temperature_node.msg import *
from temperature_node.srv import *

class ActivityManager(object):
	
	def __init__(self):
		print "Initialising node"
		# initnode 
		rospy.init_node('activity_manager')
		#rospy.wait_for_service('dht22_ask_temperature')

		self.current_plan = None
		
		# plan_stack is a json object
		self.plan_stack = list()
		self.current_action = ""

		# actuasl postion
		self.x = 0
		self.y = 0
		self.theta = 0
		
		#rospy.Publisher('mode', String)

		# service readTemp
		print "Initialising Temperature client"
		self.sense_dht22 = rospy.ServiceProxy('dht22_ask_temperature', AskTemperature)	
		
		print "Initialising ActionClient"
		# actionlin simpleMove
		self.simple_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction )
		
		print "Initialising Feedback subscriber" 
		# subscriber feedback
		self.actual_position_subscriber = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.actual_coord_callback , queue_size=1)
	
	def actual_coord_callback(self,data):
		self.x = data.feedback.base_position.pose.position.x
		self.y = data.feedback.base_position.pose.position.y
		self.theta = 0#data.feedback.base_position.pose.position.z
	
	def where_am_i(self):
		# where are you
		return  str(self.x) + ", "+str(self.y)+", "+ str(self.theta)
						
	def cancel_goal(self):
		self.simple_action_client.cancel_all_goals() 
		print "I am stopping..."
	
	def is_idle(self):
		if self.current_plan == None: return False
		else: return False

	def execute(self,action):
		if action['name'] == "goto":
			self.goto(action)		
		elif 'temp' in action['name']:
			temp = self.read_temperature()
			print self.update_temperature(temp)
		elif "wifi" in action['name']:
			self.sniff_wifi(action['iface'])
		elif "humidity" in action['name']:
			humidity = self.read_humidity()
			self.update_humidity(humidity)
		else : print "Sorry %s not recognised" %action
	
	def execute_plan(self,plan):
            	print  "Creating plan %s" % plan
		self.current_plan = plan

		self.plan_stack = json.loads(plan)
	
		while len(self.plan_stack) > 0:
			self.current_action = self.plan_stack[0]
			print "Current action %s" % self.current_action['name']
			del self.plan_stack[0] 
			time.sleep(1)
			self.execute(self.current_action)

	def goto(self,coords):
		goal = MoveBaseGoal()
   		
		#set goal
    		goal.target_pose.pose.position.x = float(coords['x'])
    		goal.target_pose.pose.position.y = float(coords['y'])
    		goal.target_pose.pose.orientation.z = math.sin(math.radians(float(coords['t']))/2)
    		goal.target_pose.pose.orientation.w = math.cos(math.radians(float(coords['t']))/2)

    		goal.target_pose.header.frame_id = 'map'
    		goal.target_pose.header.stamp = rospy.Time.now()

    		#start listener
    		self.simple_action_client.wait_for_server()

   		#send goal
    		self.simple_action_client.send_goal(goal)
		
		# finish	
		self.simple_action_client.wait_for_result(rospy.Duration.from_sec(360.0))
		
		print "Action CLient Result "+ str(self.simple_action_client.get_result()) 
		print "Going to %s, %s, %s" % (coords['x'],coords['y'], coords['t'])

	def sniff_wifi(self,interface):
		cells = list()
	        os.system("sudo iwlist %s scan > wifi.txt" % interface)
        	fopen = codecs.open("wifi.txt", "r", "utf-8")
        	print "Reading wifi signal..."
		time.sleep(2)
        	wifi=dict()
        	for line in fopen.readlines():
                	if 'Cell' in line:
                        	wifi = dict()
                        	wifi['cell']=line.lstrip()[5:7]
                        	cells.append(wifi)
                	elif 'Quality' in line:
                        	wifi['quality']=line.strip().split("  ")[0][8:]
                        	wifi['signal']=line.strip().split("  ")[1][13:]
                	elif 'ESSID' in line:
                        	wifi['name']=line.strip()[7:-1]
        	os.system("rm wifi.txt")
        	wifi_json=json.dumps(cells, sort_keys=True, indent= 2, separators=(",",":"))
		print wifi_json
		return wifi_json		

	def read_temperature(self):
    		try:
        		response = self.sense_dht22('[TEMP]')
        		return response.reply.temperature
    		except rospy.ServiceException, e:
        		print "Service call failed: %s"%e
        
	def read_humidity(self):
                try:
                        response = self.sense_dht22('[TEMP]')
                        return response.reply.humidity
                except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
	
	def update_humidity(self,hum):
		print hum

	def update_temperature(self,temp):
		print temp

from flask import Flask, request
app = Flask(__name__)
activity_manager = ActivityManager()

@app.route('/')
def index():
	return 'Please specify action\n'

@app.route('/whereAreYou', methods = ["GET"])
def where_are_you():
	if request.method == 'GET':
		# where are you
		return "Look! I am %s\n" % activity_manager.where_am_i()

@app.route('/do', methods=['POST', 'GET','DELETE'])
def do():
	if request.method == 'POST':
        	plan = request.form['p']
		if not activity_manager.is_idle():
			print plan 
			activity_manager.execute_plan(plan)
			return "", 201
        	else :
            		print  "Sorry, I am busy"
			return "", 406
    	elif request.method == 'DELETE':
		activity_manager.cancel_goal()
         	return "",204 
    	else :
        	# it is GET
        	if not activity_manager.is_idle():
			print "Doing nothing."
            		return '',204 
        	else : 
			print "Remaining %s" % activity_manager.plan_stack 
			return activity_manager.plan_stack , 200
	
if __name__ == '__main__':
	print "Starting server"
	app.run(debug=True, use_reloader=False, host='0.0.0.0')
