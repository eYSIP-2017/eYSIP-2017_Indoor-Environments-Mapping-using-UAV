#!/usr/bin/env python

import rospy
import actionlib
import math

import roslib
roslib.load_manifest('realsense_gazebo_plugin')

from geometry_msgs.msg import *
from nav_msgs.msg import *

from realsense_gazebo_plugin.msg import *
from tf.transformations import *

cur_pose = 0

# actionlib server class
class NavigateServer():
	def __init__(self):
		# start a actionlib server
		self.server = actionlib.SimpleActionServer('waypoint', waypointAction, self.navigate, False)
		self.server.start()

	def navigate(self, goal):
		# convert current pose and goal quaternions to euler form
		(goal_r, goal_p, goal_y) = euler_from_quaternion([goal.waypoint.rotation.x, goal.waypoint.rotation.y, goal.waypoint.rotation.z, goal.waypoint.rotation.w])
		(cur_r, cur_p, cur_y) = euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])
		
		# calculate errors in pose 
		error_x = cur_pose.position.x - goal.waypoint.translation.x
		error_y = cur_pose.position.y - goal.waypoint.translation.y
		error_z = cur_pose.position.z - goal.waypoint.translation.z
		error_yaw = goal_y - cur_y

		# loop until errors in translation is less than 0.1m and error in yaw  rotation is less than 0.017
		while (abs(error_x) > 0.1 or abs(error_y) > 0.1 or abs(error_z) > 0.1 or abs(error_yaw) > 0.017):
			# convert current pose and goal quaternions to euler form
			(goal_r, goal_p, goal_y) = euler_from_quaternion([goal.waypoint.rotation.x, goal.waypoint.rotation.y, goal.waypoint.rotation.z, goal.waypoint.rotation.w])
			(cur_r, cur_p, cur_y) = euler_from_quaternion([cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w])

			# calculate errors in pose
			error_x = goal.waypoint.translation.x - cur_pose.position.x
			error_y = goal.waypoint.translation.y - cur_pose.position.y 
			error_z = goal.waypoint.translation.z - cur_pose.position.z
			error_yaw = goal_y - cur_y

			if abs(error_yaw)>math.pi:
				error_yaw = error_yaw - (abs(error_yaw)/error_yaw)*math.pi*2

			# simple P controller with Kp = 3
			velocity_x = 3 * error_x
			velocity_y = 3 * error_y
			velocity_z = 3 * error_z
			velocity_yaw = error_yaw # Kp = 1

			# find required roll, pitch and altitude velocities
			velocity_pitch = velocity_x * math.cos(cur_y) + velocity_y * math.sin(cur_y)
			velocity_roll = velocity_y * math.cos(cur_y) - velocity_x * math.sin(cur_y)
			velocity_altitude = velocity_z
			
			# set the required velocities and publish them to gazebo
			cmd_vel.linear.x = velocity_pitch
			cmd_vel.linear.y = velocity_roll
			cmd_vel.linear.z = velocity_altitude
			cmd_vel.angular.z = velocity_yaw
			pub_cmdvel.publish(cmd_vel)

		# once the required goal is reached set velocities to zero
		cmd_vel.linear.x = 0
		cmd_vel.linear.y = 0
		cmd_vel.linear.z = 0
		cmd_vel.angular.z = 0
		pub_cmdvel.publish(cmd_vel)

		# set the goal as succeeded
		self.server.set_succeeded()

# callback to get current pose of the drone
def get_odom(data):
	global cur_pose
	cur_pose = data.pose.pose

if __name__ == '__main__':
	rospy.init_node('server')

	# subscriber for current pose from gazebo
	rospy.Subscriber('/ground_truth/state', Odometry, get_odom)

	# publisher to send velocity commands to gazebo
	pub_cmdvel = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
	cmd_vel = Twist()

	# initialize the server
	server = NavigateServer()
	
	rospy.spin()
