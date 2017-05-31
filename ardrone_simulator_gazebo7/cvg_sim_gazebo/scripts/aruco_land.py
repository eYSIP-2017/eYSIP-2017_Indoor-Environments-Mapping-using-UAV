#!/usr/bin/env python

import rospy
import math
from visualization_msgs.msg import Marker
from keyboard import KeyboardController
from std_msgs.msg import *
from geometry_msgs.msg import *
from ardrone_autonomy.msg import *
from tf.transformations import euler_from_quaternion

# variables to store distances from the aruco marker
distance_x = 0
distance_y = 0
distance_z = 0

# variables to store error values in the 3 axes
error_x = 0
error_y = 0
error_z = 0

# variables to store rotation of the drone along the 3 axes
rotation_x = 0
rotation_y = 0
rotation_z = 0

# variables to store the velocity commands to send to the drone
velocity_x = 0
velocity_y = 0
velocity_z = 0

# variables to store the ypr velocities to send to the drone
velocity_yaw = 0
velocity_pitch = 0
velocity_roll = 0
velocity_altitude = 0

# pid values for velocity in x-axis
kp_x = 3
ki_x = 1.6
kd_x = 0

# pid values for velocity in y-axis
kp_y = 3
ki_y = 0
kd_y = 0

# pid values for velocity in z-axis
kp_z = 0.5
ki_z = 0
kd_z = 0

# pid values for yaw of the drone
kp_yaw = 1
ki_yaw = 0

# differential component of error
d_error_x = 0
d_error_y = 0
d_error_z = 0

# previous errors
last_error_x = 0
last_error_y = 0
last_error_z = 0

# sum of errors
sum_error_x = 0
sum_error_y = 0
sum_error_z = 0
sum_error_yaw = 0

# target distances from the aruco marker
goal_x = 0
goal_y = 0
goal_z = 1.5

curr_time = 0
prev_time = 0
time_elapsed = 0

# sentinel value to skip setting pid velocities on the first iteration of the pid loop
flag = False

def callback(data):
	global distance_x, distance_y, distance_z
	global curr_time, time_elapsed, prev_time
	global error_x, error_y, error_z
	global last_error_x, last_error_y, last_error_z
	global d_error_x, d_error_y, d_error_z
	global sum_error_x, sum_error_y, sum_error_z, sum_error_yaw
	global velocity_x, velocity_y, velocity_z
	global velocity_pitch, velocity_roll, velocity_altitude, velocity_yaw
	global rotation_x, rotation_y, rotation_z
	global flag

	distance_x = data.pose.position.x
	distance_y = data.pose.position.y
	distance_z = data.pose.position.z

	(euler_r, euler_p, euler_y) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

	# difference in rotations of the drone and aruco marker
	theta = euler_y - rotation_z - math.pi/2

	curr_time = data.header.stamp.secs + data.header.stamp.nsecs * 10**-9

	if not flag:
		prev_time = curr_time
		flag = True
		return

	time_elapsed = curr_time - prev_time
	prev_time = curr_time

	error_x = distance_y - goal_x
	error_y = distance_x - goal_y
	error_z = distance_z - goal_z

	# calculate differential component of error
	d_error_x = ( error_x - last_error_x ) / time_elapsed
	d_error_y = ( error_y - last_error_y ) / time_elapsed
	d_error_z = ( error_z - last_error_z ) / time_elapsed

	# calculate sum of errors
	sum_error_x += (error_x * time_elapsed)
	sum_error_y += (error_y * time_elapsed)
	sum_error_z += (error_z * time_elapsed)
	sum_error_yaw += (rotation_z * time_elapsed)

	# pid loop for x,y,z and yaw velocities
	velocity_x = kp_x * error_x + ki_x * sum_error_x + kd_x * d_error_x
	velocity_y = kp_y * error_y + ki_y * sum_error_y + kd_y * d_error_y
	velocity_z = kp_z * error_z + ki_z * sum_error_z + kd_z * d_error_z

	velocity_pitch = velocity_x * math.cos(theta) + velocity_y * math.sin(theta)
	velocity_roll = velocity_y * math.cos(theta) - velocity_x * math.sin(theta)
	velocity_altitude = -velocity_z

	controller.cmd_vel.linear.x = velocity_roll #roll
	controller.cmd_vel.linear.y = velocity_pitch #pitch
	controller.cmd_vel.linear.z = velocity_altitude #altitude
	controller.cmd_vel.angular.z = 0 #yaw

	controller.pub_cmdvel.publish(controller.cmd_vel)

	last_error_x = error_x
	last_error_y = error_y
	last_error_z = error_z

def callback_navdata(temp):
	global rotation_x, rotation_y, rotation_z
	rotation_x = temp.rotX * math.pi / 180
	rotation_y = temp.rotY * math.pi / 180
	rotation_z = temp.rotZ * math.pi / 180

if __name__ == '__main__':
	controller = KeyboardController()
	rospy.init_node('aruco_land')
	rospy.Subscriber('/Estimated_marker',Marker,callback)
	rospy.Subscriber('/ardrone/navdata',Navdata,callback_navdata)

	# Move the drone around the aruco marker in a square
	# 0 - center of the square, 1-4 are the four corners
	while True:
		key = controller.get_key()
		if key == '0':
			goal_x = 0
			goal_y = 0
			goal_z = 1.5
		elif key == '1':
			goal_x = 0.2
			goal_y = 0.2
			goal_z = 1.5
		elif key == '2':
			goal_x = -0.2
			goal_y = 0.2
			goal_z = 1.5
		elif key == '3':
			goal_x = -0.2
			goal_y = -0.2
			goal_z = 1.5
		elif key == '4':
			goal_x = 0.2
			goal_y = -0.2
			goal_z = 1.5
		elif key == '\x03':
			break

