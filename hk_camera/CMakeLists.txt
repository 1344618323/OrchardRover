cmake_minimum_required(VERSION 2.8.3)
project(hk_camera)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  camera_info_manager
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


find_package(OpenCV)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /opt/MVS/Samples_LinuxSDK/include/
)
LINK_DIRECTORIES(
  /opt/MVS/Samples_LinuxSDK/lib/64/
)

add_executable(${PROJECT_NAME}_node
  leonard_hk_camera.cpp
  leonard_hk_camera.hpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  GCBase_gcc421_v3_0
  MvCameraControl
  GenApi_gcc421_v3_0
  MVGigEVisionSDK
  log4cpp_gcc421_v3_0
  MVRender
  Log_gcc421_v3_0
  MvUsb3vTL
  MathParser_gcc421_v3_0
  NodeMapData_gcc421_v3_0
  MediaProcess
  XmlParser_gcc421_v3_0
  X11
)
