gnome-terminal -x sh -c 'roslaunch realsense_gazebo_plugin ardrone_realsense.launch'
read -p "Press [Enter] key after gazebo launches successfully"
gnome-terminal -t 'Camera Info' -x sh -c 'rosrun realsense_gazebo_plugin pub_camera_info.py'
gnome-terminal -x sh -c 'roslaunch realsense_gazebo_plugin register.launch'
gnome-terminal -t 'Odometry' -x sh -c 'rosrun cvg_sim_gazebo ardrone_get_odometry.py'
gnome-terminal -x sh -c 'roslaunch realsense_gazebo_plugin rtabmap.launch'
gnome-terminal -t 'Keyboard' -x sh -c 'rosrun cvg_sim_gazebo keyboard.py'