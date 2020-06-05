cmake_minimum_required(VERSION 2.8.3)
project(or_test)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  cv_bridge 
  image_transport 
  sensor_msgs
  rospy
  or_msgs
  message_filters
  nav_msgs
  tf
  visualization_msgs
  interactive_markers
  rviz
  )

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


### 测试发布 ###
add_executable(testpublish
  src/testpublish.cpp
)
target_link_libraries(testpublish
  ${catkin_LIBRARIES}
)
add_dependencies(testpublish or_msgs_generate_messages)

### 测试接收 ###
add_executable(testlistener
  src/testlistener.cpp
)
target_link_libraries(testlistener
  ${catkin_LIBRARIES}
)
add_dependencies(testlistener or_msgs_generate_messages)

### 测试时间同步 ###
add_executable(timesynchronizer
  src/timesynchronizer.cpp
        )
target_link_libraries(timesynchronizer
  ${catkin_LIBRARIES}
)

### 测试tf ###
add_executable(turtle_tf_broadcaster
  src/turtle_tf_broadcaster.cpp
        )
target_link_libraries(turtle_tf_broadcaster
  ${catkin_LIBRARIES}
)

### 测试tf ###
add_executable(turtle_tf_listener
  src/turtle_tf_listener.cpp
        )
target_link_libraries(turtle_tf_listener
  ${catkin_LIBRARIES}
)

### 测试opencv ###
add_executable(robot_vision
  src/robot_vision.cpp
        )
target_link_libraries(robot_vision
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

### 测试visualization_msgs::Marker ###
add_executable(visual_test
        src/visual_test.cpp
        )
target_link_libraries(visual_test
        ${catkin_LIBRARIES}
        )

### 测试python ###
catkin_install_python(PROGRAMS
  scripts/testpylistener.py
  scripts/testpytalker.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
