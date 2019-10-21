cmake_minimum_required(VERSION 2.8.3)
project(or_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  cv_bridge 
  image_transport 
  sensor_msgs)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(testpublish
  src/testpublish.cpp
)
target_link_libraries(testpublish
  ${catkin_LIBRARIES}
)

add_executable(testlistener
  src/testlistener.cpp
)
target_link_libraries(testlistener
  ${catkin_LIBRARIES}
)

add_executable(robot_vision
  src/robot_vision.cpp
        )
target_link_libraries(robot_vision
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
