#!/usr/bin/env python

import rospy
import rosmsg
import genmsg
import rospkg
import os
from catkin.find_in_workspaces import find_in_workspaces
from RosMessage import RosMessage

_catkin_workspace_to_source_spaces = {}
_catkin_source_path_to_packages = {}

def _get_package_paths(pkgname, rospack):
    paths = []
    path = rospack.get_path(pkgname)
    paths.append(path)
    results = find_in_workspaces(search_dirs=['share'], project=pkgname, first_match_only=True, workspace_to_source_spaces=_catkin_workspace_to_source_spaces, source_path_to_packages=_catkin_source_path_to_packages)
    
    if results and results[0] != path:
        paths.append(results[0])
    return paths

if __name__ == "__main__":
	topics = rospy.get_published_topics("/")
	context = genmsg.MsgContext.create_default()

	rospack = rospkg.RosPack()
    	search_path = {}
    	for p in rospack.list():
        	package_paths = _get_package_paths(p, rospack)
        	search_path[p] = [os.path.join(d, 'msg') for d in package_paths]

	for topic in topics:
		t = topic[0]
		msg = topic[1]
		
		msg_obj = RosMessage(msg, t)

		ret = msg_obj.get_all_children_msgs()
		
		print msg_obj.get_complete_type()

		for r in ret:
			print "\t%s" % r.get_complete_type()

		#break
		#print rosmsg.get_msg_text(msg, True)
		#message = Message(msg)
		#print message.__dict__


