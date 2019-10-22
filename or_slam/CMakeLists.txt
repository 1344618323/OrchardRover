cmake_minimum_required(VERSION 3.1)
project(or_slam)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  sensor_msgs
  or_msgs
  )


catkin_package()

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(testpublish
  src/testpublish.cpp
)
target_link_libraries(testpublish
  ${catkin_LIBRARIES}
)
add_dependencies(testpublish or_msgs_generate_messages)