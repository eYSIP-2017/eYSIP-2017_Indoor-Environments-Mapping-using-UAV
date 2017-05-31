sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-indigo-gazebo7-ros-pkgs ros-indigo-gazebo7-ros-control

# Workspace name is catkin_ws
cd ~/catkin_ws
catkin_make
source devel/setup.bash