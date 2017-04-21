#! /usr/bin/env python

import rosnode
import rosmsg
import json
from RosNode import RosNode

if __name__ == "__main__":
	nodelist = rosnode.get_node_names()

	for nodename in nodelist:
		# initialise RosNode object
		node = RosNode(nodename)
		
		print json.dumps(node.get_capability_msg())
		"""msgs, srvs = rosnode info
	
		for each msg in msgs:
			rosmsg info

		for each srv in srvs:
			rossrv info """
