#! /usr/bin/env python

import rosnode
import rosmsg
import json
from RosNode import RosNode
import requests

if __name__ == "__main__":
	nodelist = rosnode.get_node_names()

	for nodename in nodelist:
		# initialise RosNode object
		node = RosNode(nodename)

		print json.dumps(node.get_capability_msg())
		
		#res = requests.get("http://localhost:5050/onto-rob-server/capabilities",data={"jsonString":node.get_capability_msg()})

		#if res.status_code == 200:
		#	print "Message successfully sent"
		#else:
		#	print "Error while sending: %s %s" % (res.status_code, res.text)

		
		"""msgs, srvs = rosnode info
	
		for each msg in msgs:
			rosmsg info

		for each srv in srvs:
			rossrv info """
