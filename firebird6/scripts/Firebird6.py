#!/usr/bin/env python

import serial
import rospy

from std_msgs.msg import String

'''
0x01 is Forward
0x02 is Reverse
0x03 is Left
0x04 is Right
0x06 is Stop
'''
commands = {'w': chr(0x94)+chr(0x01), 's': chr(0x94)+chr(0x02), 'a': chr(0x94)+chr(0x03), 'd': chr(0x94)+chr(0x04), ' ': chr(0x94)+chr(0x06)}

def callback(msg):
	# check if received key is in the commands dictionary
	if msg.data in command
		# create the required command
		buf = 'NEX' + commands[msg.data]
		# write the command to the serial port
		ser.write(bytes(buf))

if __name__ == "__main__":
	rospy.init_node('FIREBIRD6')

	# subscriber to receive commands
	rospy.Subscriber('/command', String, callback)

	# open serial port to the Firebird6 with appropriate baudrate and settings
	ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)  # open serial port
	
	rospy.spin()