#!/usr/bin/env python

from sensor_msgs.msg import *
from standard_msgs.msg import String 
from tf.msg import tfMessage
from geometry.msgs import Point32
import rospy
import tf

got_info = False
listener = ''
info_subscriber = ''
cloud = ''
laser = ''

def frame_callback(data):
	global got_info
	global cloud
	global header
	global scan
	
	if not got_info: return
	
	current_time = rospy.Time(0)
	
	#   convert the image message to a standard opencv Mat.
	cloud.points = []
	laser.points = []
	scan.ranges = []
	scan_points = 0
	min_y = 0
	min_x = 0
	max_y = 0
	max_y = 0
	
	point = Point32() 
			

def info_callback(data):
	global listener
	global info_subscriber
	# get the K matrix
	camera_info = data.plumb_bob.K
	"""
	do stuff
	"""
	
	try :	
		#wait until the transform becomes available
		listener.waitForTransform('/world', '/kinect',rospy.Time(0),rospy.Duration(30))
		listener.lookupTransform('/world', '/kinect',rospy.Time(0))
		listener.waitForTransform('/kinect', '/laser',rospy.Time(0),rospy.Duration(30))
		listener.lookupTransform('/kinect', '/laser',rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		continue

	"""
	do eigen stuff
	"""
	# basically run this only once
	info_subscriber.shutdown()	
	got_info=True

def depth_to_laser():
	global listener
	global cloud
	global header
	global scan
	
	print "Init node"
	rospy.init_node('depth_to_laser', anonymous=True)
	
	#  makes receiving transforms easier (receives and buffers tf transformations for 10 secs)
	listener = tf.TransformListener()
	
	cloud = PointCloud()
	cloud.header.frame_id = "/kinect"
	
	laser = PointCloud()
	laser.header.frame_id = "/laser"
	
	scan = LaserScan()
	scan.header.frame_id = "/laser"
		
	#subscribers
	frame_subscriber = rospy.Subscriber('/camera/depth/image_raw', String, frame_callback, queue_size=1)
	info_subscriber = rospy.Subscriber('/camera/depth/camera_info', String, info_callback, queue_size=1)
	
	# publishers
	kinect_publisher = rospy.Publisher('/pointcloud',Point, queue_size=1)
	laser_publisher = rospy.Publisher('/laser',Point, queue_size=1)
	scan_publisher = rospy.Publisher('/scan', queue_size=1)
	
	rospy.spin()

if __name__ == "__main__":
	depth_to_laser()

