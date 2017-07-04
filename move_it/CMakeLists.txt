cmake_minimum_required(VERSION 2.8.3)
project(move_it)

find_package(catkin REQUIRED genmsg actionlib_msgs actionlib trajectory_msgs)
add_action_files(DIRECTORY .action FILES MultiDofFollowJointTrajectory.action)
generate_messages(DEPENDENCIES actionlib_msgs trajectory_msgs)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
