#!/usr/bin/env python

import sys
import curses
from geometry_msgs.msg import Twist

def keytwist(verbose):
        print "Initialising node"
        rospy.init_node('key2twist', anonymous=True)
        twister = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

	# get the curses screen window
	screen = curses.initscr()
	 
	# turn off input echoing
	curses.noecho()
	 
	# respond to keys immediately (don't wait for enter)
	curses.cbreak()
	 
	# map arrow keys to special values
	screen.keypad(True)
	screen.nodelay(1)	 
	try: 
	    screen.addstr(0,0, "Use arrow keys to dirve. Use 'a' to accelarate, 'z' to decelerate.")
	    screen.addstr(1,0, "Press 'q' or Ctrl+C to exit.")
	    line = 2
	    actual_speed = 0.1
	    last_msg = ''

	    while True:
		# reset cmd_vel to stop if no key is pressed
		twist_msg = Twist()
        	
		char = screen.getch()
		if char == ord('q'):
		    break
		elif char == curses.KEY_RIGHT:
		    twist_msg.msg.angiular.z = -1*actual_speed
		elif char == curses.KEY_LEFT:
		    twist_msg.msg.angular.z = 1*actual_speed
		elif char == curses.KEY_UP:
		    twist_msg.msg.linear.x = 1*actual_speed      
		elif char == curses.KEY_DOWN:
		    twist_msg.msg.linear.x = -1*actual_speed
		elif char == ord('a'):
		    if verbose:
			    screen.addstr(line, 0, 'Actual speed %s' % actual_speed)
		    actual_speed += 0.1
		    if actual_speed > 0.5:
			actual_speed = 0.5
		    line += 1
		elif char == ord('z'):
		    if verbose:
			    screen.addstr(line, 0, 'Actual speed %s' % actual_speed)
		    actual_speed -= 0.1
		    if actual_speed < 0.0:
			actual_speed = 0.0
		    line += 1

		if twist_msg.msg.linear.x != last_msg.msg.linear.x or twist_msg.angular.z != last_msg.angular.z:
			twister.publish(twist_msg)
        
	        last_msg = twist_msg
                rospy.sleep(0.1)
	finally:
            for i in range(0,10):
		twister.publish(Twist())
	    # shut down cleanly
	    curses.nocbreak(); screen.keypad(0); curses.echo()
	    curses.endwin()

if __name__ == "__main__":
        v = False
	if rospy.has_param("/key2twist/verbose"):
		v = bool(rospy.get_param("/key2twist/verbose")
	
	keytwist(v)
