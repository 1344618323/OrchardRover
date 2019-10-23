cmake_minimum_required(VERSION 3.1)
project(or_slam)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  tf
  nav_msgs
  message_filters
  sensor_msgs
  or_msgs
  )


catkin_package()

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(fastslam_node
  src/fastslam_node.cpp
  include/fastslam_node.h
)
target_link_libraries(fastslam_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${GLOG_LIBRARY}
)
add_dependencies(fastslam_node or_msgs_generate_messages)