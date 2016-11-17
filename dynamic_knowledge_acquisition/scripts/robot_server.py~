#!/usr/bin/env python

######
# simulator without the robot
#####

import rospy
import argparse
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
import threading
import sys
import signal
from ActivityManager import ActivityManager
from flask import Flask, request

app = Flask(__name__)
global activity_manager

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    shutdown_server()
    sys.exit(0)

def shutdown_server():
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()

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
        signal.signal(signal.SIGINT, signal_handler)
	global activity_manager

	parser = argparse.ArgumentParser(description='Robot ROS server')
	parser.add_argument("-i", "--kbip", help="the IP address of the Knowledge Base server", default="http://127.0.0.1")
	parser.add_argument("-p", "--kbport", help="the PORT of the Knowledge Base server", default="8080")
	parser.add_argument("-w", "--wifi", help="the name of the WiFi to check", default="eduroam")
	args = parser.parse_args()

	if args.kbip == "http://127.0.0.1":
		print "Using localhost as default ip for KB server"
	if args.kbport == "8080":
		print "Using 8080 as default port for KB server"
	if args.wifi == "eduroam":
		print "Using eduroam as default wifi name"

	activity_manager = ActivityManager(args.kbip, args.kbport, args.wifi)
	app.run(debug=True,use_reloader=False,threaded=True, host='0.0.0.0')
