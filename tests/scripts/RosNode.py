#! /usr/bin/env python

import rosnode
import rosservice

class RosNode(object):
	def __init__(self,name):
		self.published_topics = []
		self.subscribed_topics = []
		self.services = []
		self.name = name
		self.initialise_node()

	def initialise_node(self):
		nodeinfo = rosnode.get_node_info_description(self.name)
		lines = nodeinfo.split("\n")

		count = 0
		line = lines[count]

		while not line.startswith("Publications:") and count < len(lines):
			count += 1
			line = lines[count]
			print line

		count += 1
		line = lines[count]

		while line.strip().startswith("*") and count < len(lines):
			# new published topic
			topic = self.get_topic(line.rstrip())
			msg = self.get_msg(line.rstrip())
			pub_topic = {"topic":topic,"msg":msg}
			self.published_topics.append(pub_topic)
			print "This is a published topic: %s with msg:%s" % (topic,msg)
			count += 1
			line = lines[count]

		
		while not line.startswith("Subscriptions:") and count < len(lines):
			count += 1
			line = lines[count]
		
		count += 1
		line = lines[count]

		while line.strip().startswith("*") and count < len(lines):
			# new subscribed topic
			topic = self.get_topic(line.rstrip())
			msg = self.get_msg(line.rstrip())
			sub_topic = {"topic":topic,"msg":msg}
			self.subscribed_topics.append(sub_topic)
			print "This is a subscribed topic: %s with msg:%s" % (topic,msg)
			count += 1
			line = lines[count]

		while not line.startswith("Services:") and count < len(lines):
			count += 1
			line = lines[count]
		
		count += 1
		line = lines[count]

		while line.strip().startswith("*") and count < len(lines):
			# new subscribed topic
			srv = self.get_service(line)
			srv_type = self.get_service_type(srv)
			service = {"name":srv,"msg":srv_type}
			self.services.append(service)
			print "This is service: %s with type:%s" % (srv, srv_type)
			count += 1
			line = lines[count]

	def get_topic(self,line):
		return line.split(" ")[2]

	def get_service(self,line):
		return line.split(" ")[2]

	def get_msg(self,line):
		s = line.split(" ")
		return s[3][1:len(s[3])-1]

	def explode_topics(self):
		# recursively adding all the submessages of a message
		return		

	def get_service_type(self,srv):
		return rosservice.get_service_type(srv)

	def get_capability_msg(self):
		ret = []
		for pt in self.published_topics:
			to_add = {"method":"msg","act":"pub","node":self.name,"topic":pt["topic"],"msg":pt["msg"]}
			ret.append(to_add)

		for st in self.published_topics:
			to_add = {"method":"msg","act":"sub","node":self.name,"topic":st["topic"],"msg":st["msg"]}
			ret.append(to_add)
			
		for srv in self.services:
			to_add = {"method":"srv","node":self.name,"srv":srv["name"],"msg":srv["msg"]}
			ret.append(to_add)

		return ret
