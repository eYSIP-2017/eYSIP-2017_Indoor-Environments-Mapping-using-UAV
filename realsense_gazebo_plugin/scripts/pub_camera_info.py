#!/usr/bin/env python

import rospy
from sensor_msgs.msg import *

# Initialize a CameraInfo object to store camera calibration parameters and metadata
color_cam_info = CameraInfo()
depth_cam_info = CameraInfo()

# Set camera metadata for color camera
color_cam_info.header.frame_id = 'color'
color_cam_info.height = 480
color_cam_info.width = 640
color_cam_info.distortion_model = 'plumb_bob'

# Set camera calibration parameters for color camera
color_cam_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
color_cam_info.K = [614.408935546875, 0.0, 0.0, 0.0, 619.9131469726562, 0.0, 0.0, 0.0, 1.0]
color_cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
color_cam_info.P = [614.408935546875, 0.0, 0.0, 0.0, 0.0, 619.9131469726562, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]

# Set camera metadata for depth camera
depth_cam_info.header.frame_id = 'depth'
depth_cam_info.height = 480
depth_cam_info.width = 640
depth_cam_info.distortion_model = 'plumb_bob'

# Set camera calibration parameters for depth camera
depth_cam_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
depth_cam_info.K = [593.4848022460938, 0.0, 0.0, 0.0, 593.4848022460938, 0.0, 0.0, 0.0, 1.0]
depth_cam_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
depth_cam_info.P = [593.4848022460938, 0.0, 0.0, 0.0, 0.0,  593.4848022460938, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]

rospy.init_node('CameraInfo')
pub_color_info = rospy.Publisher('/realsense/camera/color/camera_info',CameraInfo,queue_size=1)
pub_depth_info = rospy.Publisher('/realsense/camera/depth/camera_info',CameraInfo,queue_size=1)

# Publish color camera info when an image is recieved, thereby synchronizing the image and info
def color_callback(color_image):
	color_cam_info.header = color_image.header
	pub_color_info.publish(color_cam_info)

# Publish depth camera info when an image is recieved, thereby synchronizing the image and info
def depth_callback(depth_image):
	depth_cam_info.header = depth_image.header
	pub_depth_info.publish(depth_cam_info)	

rospy.Subscriber('/realsense/camera/color/image_raw',Image,color_callback)
rospy.Subscriber('/realsense/camera/depth/image_raw',Image,depth_callback)

while not rospy.is_shutdown():
	rospy.spin()
