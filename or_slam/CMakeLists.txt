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

add_subdirectory(fastslamalgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node
        src/slam_node.cpp
        include/slam_node.h
)
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${GLOG_LIBRARY}
        fastslamalgs
)
add_dependencies(${PROJECT_NAME}_node  or_msgs_generate_messages)

add_executable(fastslam_test
        src/fastslam_test.cpp
        include/fastslam_test.h
        )
target_link_libraries(fastslam_test
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${GLOG_LIBRARY}
        fastslamalgs
        )
