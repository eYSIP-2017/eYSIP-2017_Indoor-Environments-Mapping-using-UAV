# launch file to start the kinect camera
gnome-terminal -x sh -c 'roslaunch freenect_launch freenect.launch depth_registration:=true'

sleep 3 # wait for 3 seconds for the camera to load

# launch RTAB-Map package
gnome-terminal -x sh -c 'roslaunch rtabmap_ros rgbd_mapping.launch'

# start the kinect_aux_node to control the kinect cameras tilt angle
rosrun kinect_aux kinect_aux_node&

# start the script to control the Firebird6 movement
rosrun realsense_gazebo_plugin Firebird6.py
