#!/usr/bin/env python

import rospy
import math
from visualization_msgs.msg import Marker
from keyboard import KeyboardController
from std_msgs.msg import *
from geometry_msgs.msg import *
from ardrone_autonomy.msg import *
from tf.transformations import euler_from_quaternion

distance_x = 0
distance_y = 0
distance_z = 0

error_x = 0
error_y = 0
error_z = 0

rotation_x = 0
rotation_y = 0
rotation_z = 0

velocity_x = 0
velocity_y = 0
velocity_z = 0

velocity_yaw = 0
velocity_pitch = 0
velocity_roll = 0
velocity_height = 0

kp_x = 1
ki_x = 0
kd_x = 0

kp_y = 1
ki_y = 0
kd_y = 0

kp_z = 1
ki_z = 0
kd_z = 0

kp_yaw = 1
ki_yaw = 0.2

d_error_x = 0
d_error_y = 0
d_error_z = 0

last_error_x = 0
last_error_y = 0
last_error_z = 0

sum_error_x = 0
sum_error_y = 0
sum_error_z = 0
sum_error_yaw = 0

goal_x = 1.5
goal_y = 0
goal_z = 0

curr_time = 0
prev_time = 0
time_elapsed = 0

flag = False

def callback(data):
	global distance_x, distance_y, distance_z
	global curr_time, time_elapsed, prev_time
	global error_x, error_y, error_z
	global last_error_x, last_error_y, last_error_z
	global d_error_x, d_error_y, d_error_z
	global sum_error_x, sum_error_y, sum_error_z, sum_error_yaw
	global velocity_x, velocity_y, velocity_z
	global velocity_pitch, velocity_roll, velocity_height, velocity_yaw
	global rotation_x, rotation_y, rotation_z
	global flag

	distance_x = data.pose.position.x
	distance_y = data.pose.position.y
	distance_z = data.pose.position.z

	(euler_r, euler_p, euler_y) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])

	# print euler_r, euler_p, euler_y

	theta = euler_p - rotation_z - math.pi

	curr_time = data.header.stamp.secs + data.header.stamp.nsecs * 10**-9

	if not flag:
		prev_time = curr_time
		flag = True
		return

	time_elapsed = curr_time - prev_time
	prev_time = curr_time

	error_x = distance_z - goal_x
	error_y = distance_y - goal_y
	error_z = distance_x - goal_z

	print error_x, error_y, error_z, theta

	d_error_x = ( error_x - last_error_x ) / time_elapsed
	d_error_y = ( error_y - last_error_y ) / time_elapsed
	d_error_z = ( error_z - last_error_z ) / time_elapsed

	sum_error_x += (error_x * time_elapsed)
	sum_error_y += (error_y * time_elapsed)
	sum_error_z += (error_z * time_elapsed)
	sum_error_yaw += (rotation_z * time_elapsed)

	# print distance_x, distance_y, distance_z
	# print sum_error_x, sum_error_y, sum_error_z, time_elapsed


	velocity_x = kp_x * error_x + ki_x * sum_error_x + kd_x * d_error_x
	velocity_y = kp_y * error_y + ki_y * sum_error_y + kd_y * d_error_y
	velocity_z = kp_z * error_z + ki_z * sum_error_z + kd_z * d_error_z

	# velocity_pitch = velocity_x * math.cos(theta) + velocity_y * math.sin(theta)
	# velocity_roll = velocity_y * math.cos(theta) - velocity_x * math.sin(theta)
	# velocity_height = -velocity_z

	velocity_yaw = kp_yaw * rotation_z + ki_yaw * sum_error_yaw

	controller.cmd_vel.linear.x = velocity_x
	controller.cmd_vel.linear.y = -velocity_y
	controller.cmd_vel.linear.z = -velocity_z
	controller.cmd_vel.angular.z = -velocity_yaw

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
	rospy.spin()

