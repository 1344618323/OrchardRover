cmake_minimum_required(VERSION 2.8.3)
project(or_detection)
find_package(catkin REQUIRED COMPONENTS 
  rospy
  message_generation 
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/trunk_detection.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)