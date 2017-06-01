#!/usr/bin/env python

import rospy
from nav_msgs.msg import *
from gazebo_msgs.msg import *
import tf

odom_data = Odometry()

odom_data.header.frame_id = 'odom'
odom_data.child_frame_id = 'quadrotor'

rospy.init_node('OdometryData')
pub_odom = rospy.Publisher('/ardrone/odometry', Odometry, queue_size=1)
pub_tf = tf.TransformBroadcaster()

def callback(data):
	# print data
	odom_data.header.stamp = rospy.Time.now()
	quadrotor_index = data.name.index("quadrotor")
	odom_data.pose.pose = data.pose[quadrotor_index]
	odom_data.twist.twist = data.twist[quadrotor_index]
	pub_odom.publish(odom_data)

rospy.Subscriber('/gazebo/model_states', ModelStates, callback)

rate = rospy.Rate(20)

while not rospy.is_shutdown():
	pub_tf.sendTransform((odom_data.pose.pose.position.x, odom_data.pose.pose.position.y, odom_data.pose.pose.position.z), (odom_data.pose.pose.orientation.x, odom_data.pose.pose.orientation.y, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w), rospy.Time.now(), 'quadrotor', 'odom')
	rate.sleep()