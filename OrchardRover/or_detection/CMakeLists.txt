cmake_minimum_required(VERSION 3.1)
project(or_detection)
find_package(catkin REQUIRED COMPONENTS 
  rospy
  message_generation 
  std_msgs
  or_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS         
  scripts/test_trunk_detection.py
  scripts/trunk_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)