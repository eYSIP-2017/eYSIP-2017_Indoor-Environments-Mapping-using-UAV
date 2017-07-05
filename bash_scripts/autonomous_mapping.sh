# Launchh Gazebo7 and load the simulated world
gnome-terminal -t 'Gazebo' -x sh -c 'roslaunch realsense_gazebo_plugin ardrone_realsense.launch'
read -p "Press [Enter] key after gazebo launches successfully"

# publish fake depth and color camera info for the simulated cameras
gnome-terminal -t 'Camera Info' -x sh -c 'rosrun realsense_gazebo_plugin pub_camera_info.py'

# register the depth image to the color image frame
gnome-terminal -t 'Register' -x sh -c 'roslaunch realsense_gazebo_plugin register.launch'

# run script to fetch pose of the ardrone in Gazebo and publish the tf
gnome-terminal -t 'Odometry' -x sh -c 'rosrun cvg_sim_gazebo ardrone_get_odometry.py'

# Launch nodelet to convert depth and color image to pointcloud
gnome-terminal -t 'PointCloud' -x sh -c 'roslaunch realsense_gazebo_plugin rtabmap_pcl.launch'

# Launch the MoveIt! path planner
gnome-terminal -t 'MoveIt' -x sh -c 'roslaunch move_it moveit.launch'

# Start the actionlib server to execute the waypoints from MoveIt!
gnome-terminal -t 'Server' -x sh -c 'rosrun realsense_gazebo_plugin server.py'

# Start the actionlib client to execute the waypoints from MoveIt!
gnome-terminal -t 'Client' -x sh -c 'rosrun realsense_gazebo_plugin client.py'

# Start the keyboard tele-op to control the drone in Gazebo
gnome-terminal -t 'Keyboard' -x sh -c 'rosrun cvg_sim_gazebo keyboard.py'

# start the script to send goals to MoveIt!
rosrun realsense_gazebo_plugin send_goal.py

# Launch the FBETServer
gnome-terminal -t 'FBET' -x sh -c 'roslaunch realsense_gazebo_plugin fbet.launch'
