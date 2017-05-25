#!/usr/bin/env python
#import rospy and msg libraries
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from ardrone_autonomy.msg import *

import sys, select, termios, tty

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
	rospy.init_node('NODE')
	controller = KeyboardController()
	while True:
		key = controller.get_key()

		controller.cmd_vel.linear.x = 0
		controller.cmd_vel.linear.y = 0
		controller.cmd_vel.linear.z = 0
		controller.cmd_vel.angular.z = 0

		if key in moveBindings.keys():
			controller.cmd_vel.linear.x = moveBindings[key][2]
			controller.cmd_vel.linear.y = moveBindings[key][3]
			controller.cmd_vel.linear.z = moveBindings[key][0]
			controller.cmd_vel.angular.z = moveBindings[key][1]

			controller.pub_cmdvel.publish(controller.cmd_vel)

		elif key == 'y':
			controller.pub_takeoff.publish(Empty())
		
		elif key == 'h':
			controller.pub_land.publish(Empty())

		elif key == ' ':
			controller.pub_reset.publish(Empty())

		elif key == '\x03':
			break
		else:
			controller.pub_cmdvel.publish(controller.cmd_vel)