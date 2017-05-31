#!/usr/bin/env python

#import rospy and msg libraries
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from ardrone_autonomy.msg import *

import sys, select, termios, tty

# dictionary to store velocity commands for movement of the drone
moveBindings = {
'q':(1,0,0,0),
'a':(-1,0,0,0),
'w':(0,1,0,0),
'r':(0,-1,0,0),
'e':(0,0,1,0),
'd':(0,0,-1,0),
's':(0,0,0,1),
'f':(0,0,0,-1),
}

settings = termios.tcgetattr(sys.stdin)

class KeyboardController():
	def __init__(self):
		self.cmd_vel = Twist()
		self.pub_land = rospy.Publisher('/ardrone/land',Empty,queue_size=1)
		self.pub_takeoff = rospy.Publisher('/ardrone/takeoff',Empty,queue_size=1)
		self.pub_reset = rospy.Publisher('/ardrone/reset',Empty,queue_size=1)
		self.pub_cmdvel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

	# function to get the current key being pressed
	def get_key(self):
	    tty.setraw(sys.stdin.fileno())
	    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	    if rlist:
	        key = sys.stdin.read(1)
	    else:
	        key = ''
	    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	    return key

if __name__=="__main__":
	rospy.init_node('KeyboardController')

	# initialize a KeyboardController object
	controller = KeyboardController()
	while True:

		# get the current key being pressed
		key = controller.get_key()

		controller.cmd_vel.linear.x = 0
		controller.cmd_vel.linear.y = 0
		controller.cmd_vel.linear.z = 0
		controller.cmd_vel.angular.z = 0

		# publish the velocity command if key pressed corresponds to a command
		if key in moveBindings.keys():
			controller.cmd_vel.linear.x = moveBindings[key][2]
			controller.cmd_vel.linear.y = moveBindings[key][3]
			controller.cmd_vel.linear.z = moveBindings[key][0]
			controller.cmd_vel.angular.z = moveBindings[key][1]
			controller.pub_cmdvel.publish(controller.cmd_vel)

		# press key 'y' to take-off
		elif key == 'y':
			controller.pub_takeoff.publish(Empty())
		
		# press key 'h' to land
		elif key == 'h':
			controller.pub_land.publish(Empty())

		# press the spacebar to reset the drone
		elif key == ' ':
			controller.pub_reset.publish(Empty())

		elif key == '\x03':
			break

		# pressing any other key sends 0 velocity
		else:
			controller.pub_cmdvel.publish(controller.cmd_vel)