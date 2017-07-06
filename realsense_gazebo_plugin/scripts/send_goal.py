#!/usr/bin/env python

# To use the python interface to move_group, import the moveit_commander module.
import rospy
import sys
import moveit_commander

import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from moveit_msgs.msg import RobotTrajectory

import math

goal = 0
curr_pose = 0

# function to initialize move_group node and plan the trajectories for a given goal
def move_group_interface():
	# First initialize moveit_commander and rospy.
	moveit_commander.roscpp_initialize(sys.argv)

	# Instantiate a MoveGroupCommander object.  This object is an interface to one group of joints.
	group = moveit_commander.MoveGroupCommander("quad_base")

	#set workspace for the planner
	group.set_workspace([-3.5,-3.5,0,3.5,3.5,3])
	
	# set the goal and plan
	group.set_joint_value_target(goal)
	plan = group.plan()
	
	#if plan is empty, tell FBETServer the goal is invalid
	empty = RobotTrajectory()
	if plan == empty:
		print "Empty Plan"
		feedback = String()
		feedback.data = ""
		pub_feedback.publish(feedback)

# callback to get the current pose of the drone
def get_odom(data):
	global curr_pose
	curr_pose = data

# callback to set the goal and send the goal to MoveIt
def set_goal(data):
	global goal

	x = data.position.x
	y = data.position.y
	z = data.position.z
	angle = data.orientation.x # sent in euler form

	if x < curr_pose.pose.pose.position.x:
		angle = angle + math.pi

	quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
	goal = [x,y,z,quaternion[0],quaternion[1],quaternion[2],quaternion[3]]

	try:
		move_group_interface()
	except rospy.ROSInterruptException:
		pass

if __name__=='__main__':
	rospy.init_node('move_group_python_interface', anonymous=True)
	
	# subscribe to goal from FBETServer
	rospy.Subscriber('/goal_autonomous', Pose, set_goal)
	
	# subscribe to current pose from gazebo
	rospy.Subscriber('/ground_truth/state', Odometry, get_odom)

	# publisher to inform FBETServer about invalid goal
	pub_feedback = rospy.Publisher('/fbet/feedback', String, queue_size=1)

	rospy.spin()

