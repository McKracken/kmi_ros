#!/usr/bin/env python

from irobotcreate2.msg import Bumper
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf.msg import tfMessage
from tf import transformations
import random 
import rospy

bumper = 0
twister = 0
coord_X = 0
coord_Y = 0
coord_Z = 0
last_turned = ''
quaternion = ''



def stop():
	global twister
	twist_msg = Twist()
	twister.publish(twist_msg)

def turn(direction,variation_angle):
	global twister
	global quaternion
	global last_turned
	twist_msg = Twist()
	
	initial_position = transformations.euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])[2]*57.3
	if  initial_position < 0 : 
		initial_position = initial_position+360
	
	spun_deg = 0	

	if direction == 'left':
		twist_msg.angular.z = 1
	
		while spun_deg < variation_angle:
			twister.publish(twist_msg)
			current_position = transformations.euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])[2]*57.3
			if current_position < 0 : 
				current_position = current_position+360
			diff = current_position-initial_position
			if diff < 0:
				diff = 360-diff
			spun_deg=spun_deg+diff
			print "left s", spun_deg,"d ", diff
			initial_position = current_position
			rospy.sleep(0.1)
	else :
		#turn right	
		twist_msg.angular.z = -1
				
		while spun_deg < variation_angle:	
			twister.publish(twist_msg)
			current_position = transformations.euler_from_quaternion([quaternion.x,quaternion.y,quaternion.z,quaternion.w])[2]*57.3
			# that's the conversion Euler to 360deg format
			if  current_position < 0 :  
				current_position = current_position+360
			
			
			diff = initial_position-current_position			
			if diff < 0 :
				diff = diff+360
			spun_deg = spun_deg+diff 
			
			print "right s", spun_deg,"d ", diff
			initial_position = current_position
			rospy.sleep(0.1)
				
	last_turned = direction		
	#rospy.sleep(0.1)

def forward():
	global twister
	twist_msg = Twist()
	twist_msg.linear.x = 0.2
	twister.publish(twist_msg)
	rospy.sleep(0.1)

def back(time): 
	global twister
	twist_msg = Twist()
	twist_msg.linear.x = -0.1
	for i in range(time):
		twister.publish(twist_msg)
		rospy.sleep(0.1)

def drive_randomly(data):
	global last_turned
	global quaternion	
	left = data.left.state
	right = data.right.state
	
	if left and not right:
		back(8)
		turn("right",random.randint(20,45))
		stop()	
	elif right and not left :
		back(8)
		turn("left",random.randint(20,45))
		stop()
	elif right and left : 
		back(10) # back and then turn 
		if last_turned == 'right':
			turn("right",random.randint(45,75))
		elif last_turned == 'left' :
			turn("left",random.randint(45,75))
		else:
			i = random.randint(0,1)
			if i == 0 : 
				turn('left',random.randint(45,75))
			else : 
				turn('right',random.randint(45,75))
			
		stop()
	else : forward()
		
def random_driver():
	print "Initialising node"
	rospy.init_node('random_driver', anonymous=True)
	rospy.on_shutdown(shutdown)

	global twister 
	global bumper
	
	print "Switching to Roomba's full mode"
	mode = rospy.Publisher('/mode', String, queue_size=1)
	s = String()
	s.data = "full"
	rospy.sleep(1)
	mode.publish(s)
	
	print "Initialising publishers and subscribers."
	twister = rospy.Publisher('/cmd_vel', Twist, queue_size=1)	
	rospy.Subscriber('/tf', tfMessage , position , queue_size=1)
	bumper = rospy.Subscriber('/bumper', Bumper , drive_randomly, queue_size=1 )
	
	rospy.spin()

def position(data):
        global coord_X
        global coord_Y
	global coord_Z
	global quaternion 

        if len(data.transforms) > 0:
                coord_X = data.transforms[0].transform.translation.x 
                coord_Y = data.transforms[0].transform.translation.y
                coord_Z = data.transforms[0].transform.rotation.z
		quaternion = data.transforms[0].transform.rotation

def shutdown():
	global bumper
	global twister
	bumper.unregister()
	for i in range(20):
		twister.publish(Twist())
	twister.unregister()
	print "Stopping..."

if __name__ == "__main__":
	print "Starting..."
	random_driver()
