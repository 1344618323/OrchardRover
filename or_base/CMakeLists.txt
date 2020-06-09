cmake_minimum_required(VERSION 2.8.3)

project(or_base)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  or_msgs
  sensor_msgs
)

catkin_package(
)


# 通讯节点
add_executable(or_base_node
        serial_comm_node.cpp
        serial_comm_node.h
        writetxt.h
        ${CMAKE_CURRENT_SOURCE_DIR}/hardware/serial_device.cpp
        )

target_link_libraries(or_base_node
        PUBLIC
        ${catkin_LIBRARIES}
        ${GLOG_LIBRARY}
        ${EIGEN3_LIBRARIES})

target_include_directories(or_base_node
        PUBLIC
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/hardware
        ${EIGEN3_INCLUDE_DIR}
        )
add_dependencies(or_base_node or_msgs_generate_messages)




