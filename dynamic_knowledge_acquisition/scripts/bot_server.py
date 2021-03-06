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
import urllib,urllib2
import requests
from temperature_node.msg import *
from temperature_node.srv import *
import random
import tf

_KB_SERVER_IP = "http://127.0.0.1"
_ACTION_CLIENT_MAX_DURATION = 360.0

class ActivityManager(object):
	
	def __init__(self):
		print "Initialising node"
		# initnode 
		rospy.init_node('activity_manager')
		#rospy.wait_for_service('dht22_ask_temperature')
		self.listener = tf.TransformListener()		
		self.current_plan = []
		
		# plan_stack is a json object
		self.plan_stack = list()
		self.current_action = ""

		try:
			(trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
			self.x = trans[0]
	                self.y = trans[1]
        	        self.theta = 0.0 #this must be converted from quaternion
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			self.x = 0.0
	                self.y = 0.0
        	        self.theta = 0.0

		#rospy.Publisher('mode', String)

		# service readTemp
		#print "Initialising Temperature client"
		# self.sense_dht22 = rospy.ServiceProxy('dht22_ask_temperature', AskTemperature)	
		
		print "Initialising ActionClient"
		# actionlin simpleMove
		self.simple_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction )
		
		print "Initialising Feedback subscriber" 
		# subscriber feedback
		self.actual_position_subscriber = rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.actual_coord_callback , queue_size=1)
	
	def actual_coord_callback(self,data):
		self.x = data.feedback.base_position.pose.position.x
		self.y = data.feedback.base_position.pose.position.y
		self.theta = data.feedback.base_position.pose.position.z
	
	def where_am_i(self):
		# where are you
		try:
                        (trans,rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                        self.x = trans[0]
                        self.y = trans[1]
                        self.theta = 0.0 #this must be converted from quaternion
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        print "Using the MoveBase coordinates"
			pass
		
		position = {}
		position['current_position']={'x' : str(self.x), 'y': str(self.y), 'theta': str(self.theta)}
		json_pos = json.dumps(position)
		#print "Returning %s" % json_pos
		return  json_pos

	def get_simpleclient_state(self,state):
		if state == actionlib.GoalStatus.SUCCEEDED:
			return "succeeded"
		elif state == actionlib.GoalStatus.ACTIVE:
			return "active"
		elif state == actionlib.GoalStatus.PENDING:
			return "pending"
		elif state == actionlib.GoalStatus.RECALLED:
			return "recalled"
		elif state == actionlib.GoalStatus.REJECTED:
			return "rejected"
		elif state == actionlib.GoalStatus.PREEMPTED:
			return "preempted"
		elif state == actionlib.GoalStatus.ABORTED:
			return "aborted"
		elif state == actionlib.GoalStatus.LOST:
			return "lost"
						
	def cancel_goal(self):
		print "I am stopping..."
		self.simple_action_client.cancel_all_goals()
		self.simple_action_client.wait_for_result(rospy.Duration.from_sec(_ACTION_CLIENT_MAX_DURATION))
		
		cur_simpleclient_state = self.simple_action_client.get_state()
		simpleclient_state_string = self.get_simpleclient_state(cur_simpleclient_state)	

		if cur_simpleclient_state == actionlib.GoalStatus.SUCCEEDED:
			print "All goals successfully cancelled"
			self.current_plan = []
			self.plan_stack = list()
			self.current_action = ""
		else:
			print "Problem while performing aborting the plan: %s" % simpleclient_state_string
			self.current_action = ""			
			self.current_plan = []
			self.plan_stack = list()

		#print "Plan stack size after cancellation: %s" % len(self.plan_stack)
		time.sleep(1)
			
	

	def is_idle(self):
		time.sleep(0.5)
		#print "You ask me if I'm busy. My plan stack is of %s items" % len(self.plan_stack)
		if len(self.plan_stack) == 0: 
			# then it is doing nothing
			return False
		else: 
			return True


	def execute(self,action):
		if action['name'] == "goto":
			self.goto(action)
			#print "I should start wait here for action %s" % action
			self.simple_action_client.wait_for_result(rospy.Duration.from_sec(_ACTION_CLIENT_MAX_DURATION))
			#print "And now I finish my wait for action %s" % action
		elif 'temp' in action['name']:
			temp = self.read_temperature()
			self.update('temperature',temp)
			#time.sleep(0.5)
		elif "wifi" in action['name']:
			wifis = self.sniff_wifi(action['iface'])

			for wifi in wifis:
				#if wifi['name'] == "WireCasaLess": #TODO back to eduroam/_The Cloud
				if wifi['name'] == "eduroam":
					signal_digit = wifi['signal'].split()[0]
					self.update("wifi",signal_digit)	
					#time.sleep(0.5)
					break
		elif "humidity" in action['name']:
			humidity = self.read_humidity()
			self.update('humidity',humidity)
			#time.sleep(0.5)
		elif "count_people" in action['name']:
			people_count = self.count_people()
			self.update('count_people',people_count)
			#time.sleep(0.5)
 		else : 
			print "Sorry, action %s not recognised" %action
#		print "%s action done" % str(action)
		time.sleep(0.5)
	
	def execute_plan(self):
            	#print  "Creating plan %s" % plan
		#self.current_plan = plan
		#self.plan_stack = json.loads(plan)
		#print self.plan_stack
		while len(self.plan_stack) > 0:
			self.current_action = self.plan_stack[0]
			#print "Current action %s" % self.current_action['name']
			#del self.plan_stack[0] 
			time.sleep(0.5)
			self.execute(self.current_action)
			try:
				del self.plan_stack[0] 
			except IndexError:
				print "Trying deleting plan from an empty stack. Probably plan aborted."
				break
	
	def goto(self,coords):
		global _ACTION_CLIENT_MAX_DURATION
		print "Going to %s, %s, %s" % (coords['x'],coords['y'], coords['t'])
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
		
		# waiting
		

		#cur_simpleclient_state = self.simple_action_client.get_state()
		#simpleclient_state_string = self.get_simpleclient_state(cur_simpleclient_state)	

		#if cur_simpleclient_state == actionlib.GoalStatus.SUCCEEDED:
		#	print "Goto action completed successfully"
		#else:
		#	print "Problem while performing the goto action: %s" % simpleclient_state_string
		#	print "Plan cancellation needed"
			#maybe I want to jump to the next move action			
		#	self.cancel_goal()


	def sniff_wifi(self,interface):		
        	print "Reading wifi signal on %s..." % interface
		cells = list()
	        os.system("sudo iwlist %s scan > wifi.txt" % interface)
		try:
        		fopen = codecs.open("wifi.txt", "r", "utf-8")
		
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
			return cells
		except IOError:
			print "Error while reading from the interface %s" % interface
		
	def count_people(self):
		print "Counting people"
		return random.randint(0,7)

	def read_temperature(self):
		print "Reading Temperature"
		return round(random.uniform(18,23),2) # TODO activare real sensor
    		try:
        		response = self.sense_dht22('[TEMP]')
        		return response.reply.temperature
    		except rospy.ServiceException, e:
        		print "Service call failed: %s"%e
        
	def read_humidity(self):
		return round(random.uniform(40,60),2) # TODO activate real sensor
                try:
                        response = self.sense_dht22('[TEMP]')
                        return response.reply.humidity
                except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
	
	def update(self,field,value):
		#print field,':',value
		params = {'field': field, 'value': value, 'x': self.x, 'y': self.y, 'theta': self.theta}
		params_json = json.dumps(params, sort_keys=True, indent= 2, separators=(",",":"))
		params_encoded = urllib.urlencode({'d': params_json})
		try:
			#req = requests.post("http://137.108.127.33:8080/bot/write", data=params_encoded)
			#print req.text,req.status_code
			url = "%s:8080/bot/write" % _KB_SERVER_IP
			print "Update, opening connection with %s" % url
			print "Updating %s with value %s" % (field, value)
			f = urllib.urlopen(url, params_encoded)
			#print f.getcode()
			f.close()
			print "Update, close connection."
			return True
		
		except IOError, e:
			print "Error %s" %str(e)
			return False 
		
	def plan_pretty_print(self):
		print ""
		print "PLAN:"
		i = 1
		for plan in self.plan_stack:
			action = plan["name"]
			
			if action == "goto":
				dest_x = plan["x"]
				dest_y = plan["y"]
				print "%s - %s X=%s Y=%s" % (i,action,dest_x,dest_y)
			elif action == "sniff_wifi":
				iface = plan["iface"]
				print "%s - %s on interface %s" % (i,action,iface)
			else:
				print "%s - %s" % (i,action)
			i += 1
		print ""

import threading
import sys
from flask import Flask, request

if len(sys.argv) > 1:
	_KB_SERVER_IP = sys.argv[1]

app = Flask(__name__)
activity_manager = ActivityManager()

@app.route('/')
def index():
	return 'Please specify action\n'

@app.route('/shutdown', methods=['POST'])
def shutdown():
    shutdown_server()
    return 'Server shutting down...'

@app.route('/whereareyou', methods = ["GET"])
def where_are_you():
	if request.method == 'GET':
		# where are you
		return '%s\n' % activity_manager.where_am_i(),200

@app.route('/currentplan', methods = ["GET"])
def get_current_paln():
        if request.method == 'GET':
                # get plan
		if len(activity_manager.current_plan) > 0:
	                return '%s\n' % activity_manager.current_plan,200
		else:
			return '[]\n', 200

@app.route('/update', methods = ['POST'])
def update():
	# outdated 'field', 'action', 'value', 'x' , 'y'
	outdated = request.form['p']
	if update_info(outdated):
		return "",200
	else : return 204 # TODO : handle errors


def shutdown_server():
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()

def execute_plan(am):
	am.execute_plan()
	
@app.route('/do', methods=['POST', 'GET','DELETE'])
def do():
	if request.method == 'POST':
        	plan = request.form['p']
		if not activity_manager.is_idle():
			#activity_manager.execute_plan(plan)
			activity_manager.current_plan = plan
			if plan == "]":
				print "Unfeasible plan"
				return "",204
			else:
	                	activity_manager.plan_stack = json.loads(plan)
				print "PLAN STACK: %s" % activity_manager.plan_stack
				activity_manager.plan_pretty_print()
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
			#print "Remaining %s" % activity_manager.plan_stack
			resp = str(activity_manager.current_action)
			resp = resp.replace("u'", "'")
			resp = resp.replace("'","\"")
			return resp+"\n", 200 
	
if __name__ == '__main__':
	app.run(debug=True,use_reloader=False,threaded=True, host='0.0.0.0')
