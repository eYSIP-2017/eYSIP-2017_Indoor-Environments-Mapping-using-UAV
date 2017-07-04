#!/usr/bin/env python

import rospy
from nav_msgs.msg import *
import tf

odom_data = Odometry()

rospy.init_node('TF')
pub_tf = tf.TransformBroadcaster()

# callback for odometry data subscriber
def callback(data):
	odom_data.pose.pose = data.pose.pose

# subscribe to odometry data published by gazebo
rospy.Subscriber('/ground_truth/state', Odometry, callback)

# 20 Hz
rate = rospy.Rate(20)

while not rospy.is_shutdown():
	# publish transform from odom frame to quadrotor frame
	pub_tf.sendTransform((odom_data.pose.pose.position.x, odom_data.pose.pose.position.y, odom_data.pose.pose.position.z), (odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w), rospy.Time.now(), 'quadrotor', 'odom')
	rate.sleep()