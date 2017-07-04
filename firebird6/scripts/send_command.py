#!/usr/bin/env python

import rospy
import sys, select, termios, tty

from std_msgs.msg import *

settings = termios.tcgetattr(sys.stdin)

curr_angle = 0.0

# callback for the kinect cameras tilt angle subscriber
def get_angle(angle):
	global curr_angle
	curr_angle = angle.data

# function to get the current key being pressed on the keyboard
def get_key():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
	rospy.init_node('Command')

	# publisher to send commands to the Firebird6
	pub_command = rospy.Publisher('/command',String,queue_size = 10)

	# publisher to control the kinect camera tilt angle
	pub_kinect = rospy.Publisher('/tilt_angle',Float64,queue_size = 1)

	# subscribe to the kinect cameras current tilt angle
	rospy.Subscriber('/cur_tilt_angle',Float64,get_angle)

	while True:
		key = get_key()

		# if Ctrl+C is pressed, exit the script
		if key == '\x03':
			break

		# increase tilt angle by 5 degrees
		elif key == 'i':
			angle = Float64()
			angle.data = curr_angle + 5

			# limit tilt angle to 31
			if angle.data > 31:
				angle.data = 31

			# if current tilt angle is invalid i.e. -64 then do nothing
			if curr_angle == -64:
				continue

			pub_kinect.publish(angle)

		# decrease tilt angle by 5 degrees
		elif key == 'k':
			angle = Float64()
			angle.data = curr_angle - 5

			# limit tilt angle to -31
			if angle.data < -31:
				angle.data = -31
			
			# if current tilt angle is invalid i.e. -64 then do nothing
			if curr_angle == -64:
				continue
			
			pub_kinect.publish(angle)

		# simply publish the key pressed
		else:
			pub_command.publish(key)