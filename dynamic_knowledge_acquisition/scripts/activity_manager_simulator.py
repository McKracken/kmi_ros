#!/usr/bin/env python

######
# simulator without the robot
#####


import rospy
import time
import json
import math
import os
import codecs
from move_base_msgs.msg import *
import actionlib
import urllib
from temperature_node.msg import *
from temperature_node.srv import *

class ActivityManager(object):
	
	def __init__(self):
		print "Initialising node"
		# initnode 
		rospy.init_node('activity_manager')
		rospy.wait_for_service('dht22_ask_temperature')

		self.current_plan = []
		
		# plan_stack is a json object
		self.plan_stack = list()
		self.current_action = ""

		# actuasl postion
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		
		#rospy.Publisher('mode', String)

		# service readTemp
		print "Initialising Temperature client"
		self.sense_dht22 = rospy.ServiceProxy('dht22_ask_temperature', AskTemperature)	
		
		print "Initialising ActionClient"
		# actionlin simpleMove
		#self.simple_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction )
		
		print "Initialising Feedback subscriber" 
		# subscriber feedback
		#self.actual_position_subscriber = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.actual_coord_callback , queue_size=1)
	
	def actual_coord_callback(self,data):
		self.x = data.feedback.base_position.pose.position.x
		self.y = data.feedback.base_position.pose.position.y
		self.theta = 0.0#data.feedback.base_position.pose.position.z
	
	def where_am_i(self):
		# where are you
		position = {}
		position['current_position']={'x' : str(self.x), 'y': str(self.y), 'theta': str(self.theta)}
		json_pos = json.dumps(position)
		return  json_pos #str(self.x) + ", "+str(self.y)+", "+ str(self.theta)
						
	def cancel_goal(self):
		#self.simple_action_client.cancel_all_goals() 
		print "I am stopping..."
		self.plan_stack = []

	
	def is_idle(self):
		if len(self.plan_stack) == 0: 
			# then it is doing nothing
			return False
		else: 
			return True

	def execute(self,action):
		if action['name'] == "goto":
			self.goto(action)		
		elif 'temp' in action['name']:
			temp = self.read_temperature()
			self.update_temperature(temp)
		elif "wifi" in action['name']:
			self.sniff_wifi(action['iface'])
		elif "humidity" in action['name']:
			humidity = self.read_humidity()
			self.update_humidity(humidity)
		else : print "Sorry %s not recognised" %action
		time.sleep(20)
	
	def execute_plan(self):
            	#print  "Creating plan %s" % plan
		#self.current_plan = plan
		#self.plan_stack = json.loads(plan)
	
		while len(self.plan_stack) > 0:
			self.current_action = self.plan_stack[0]
			print "Current action %s" % self.current_action['name']
			#del self.plan_stack[0] 
			time.sleep(1)
			self.execute(self.current_action)
			del self.plan_stack[0] 
	
	def goto(self,coords):
		"""
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
		"""
		print "Going to %s, %s, %s" % (coords['x'],coords['y'], coords['t'])
		print "Waiting 10 secs"
		time.sleep(10)

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
		params = urllib.urlencode({'humidity': hum, 'X': self.x, 'Y': self.y})
		try: 
			f = urllib.urlopen("http://www.sparqlEP.org", params)
			print f.read()
			print "Remember Sparql endpoint."
			return True
		except IOError, e:
			print "Remember Sparql endpoint."
			return False 


	def update_temperature(self,temp):
		print temp
		params = urllib.urlencode({'temperature': temp, 'X': self.x, 'Y': self.y})
		try: 
			f = urllib.urlopen("http://www.sparqlEP.org", params)
			print f.read()
			return True
		except IOError, e:
			return False 




import threading
from flask import Flask, request
app = Flask(__name__)
activity_manager = ActivityManager()

@app.route('/')
def index():
	return 'Please specify action\n'

@app.route('/whereareyou', methods = ["GET"])
def where_are_you():
	if request.method == 'GET':
		# where are you
		return '%s\n' % activity_manager.where_am_i(),200

@app.route('/update', methods = ['POST'])
def update():
	# outdated 'field', 'action', 'value', 'x' , 'y'
	outdated = request.form['p']
	if update_info(outdated):
		return "",200
	else : return 204 # TODO : handle errors


def execute_plan(am):
	am.execute_plan()
	
@app.route('/do', methods=['POST', 'GET','DELETE'])
def do():
	if request.method == 'POST':
        	plan = request.form['p']
		if not activity_manager.is_idle():
			#activity_manager.execute_plan(plan)
			activity_manager.current_plan = plan
                	activity_manager.plan_stack = json.loads(plan)
			thr = threading.Thread(target=execute_plan, args=(activity_manager,))
			thr.start()
			return "Process started: %s.\n" % thr.is_alive(), 201
        	else :
            		print  "Sorry, I am busy."
			return "Idle.\n", 406 # idle
    	elif request.method == 'DELETE':
		activity_manager.cancel_goal()
         	return "",204
    	else :
        	# it is GET
        	if not activity_manager.is_idle():
			print "Doing nothing."
            		return 'Doing nothing\n',204 
        	else : 
			print "Remaining %s" % activity_manager.plan_stack 
			resp = str(activity_manager.current_action)
			resp = resp.replace("u'", "'")
			return resp+"\n", 200 
	
if __name__ == '__main__':
	print "Starting server"
	app.run(debug=True,use_reloader=False,threaded=True, host='0.0.0.0')
