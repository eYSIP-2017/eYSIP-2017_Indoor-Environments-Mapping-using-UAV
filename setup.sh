cd ~/catkin_ws/src
git clone https://github.com/eYSIP-2017/eYSIP-2017_Indoor-Environments-Mapping-using-UAV.git
mv eYSIP-2017_Indoor-Environments-Mapping-using-UAV IndoorEnvironmentMappingUsingUAV
cd ~/catkin_ws/src/IndoorEnvironmentMappingUsingUA/bash_scripts
chmod a+x *.sh
cd ~/catkin_ws/src/IndoorEnvironmentMappingUsingUA/ardrone_simulator_gazebo7/cvg_sim_gazebo/scripts
chmod a+x *.py
cd ~/catkin_ws/src/IndoorEnvironmentMappingUsingUA/firebird6/scripts
chmod a+x *.py
cd ~/catkin_ws/src/IndoorEnvironmentMappingUsingUA/realsense_gazebo_plugin/scripts
chmod a+x *.py
echo "export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/$USER" >> ~/.bashrc
