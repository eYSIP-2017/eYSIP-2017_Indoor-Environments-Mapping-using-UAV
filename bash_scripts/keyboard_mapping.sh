# Launchh Gazebo7 and load the simulated world
gnome-terminal -x sh -c 'roslaunch realsense_gazebo_plugin house.launch'
read -p "Press [Enter] key after gazebo launches successfully"

# publish fake depth and color camera info for the simulated cameras
gnome-terminal -t 'Camera Info' -x sh -c 'rosrun realsense_gazebo_plugin pub_camera_info.py'

# register the depth image to the color image frame
gnome-terminal -x sh -c 'roslaunch realsense_gazebo_plugin register.launch'

# run script to fetch pose of the ardrone in Gazebo and publish the tf
gnome-terminal -t 'Odometry' -x sh -c 'rosrun cvg_sim_gazebo ardrone_get_odometry.py'

# build the map incrementally using the main rtabmap node
gnome-terminal -x sh -c 'roslaunch realsense_gazebo_plugin rtabmap.launch'

# Start the keyboard tele-op to control the drone in Gazebo
gnome-terminal -t 'Keyboard' -x sh -c 'rosrun cvg_sim_gazebo keyboard.py'