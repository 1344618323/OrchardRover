cmake_minimum_required(VERSION 3.1)
project(or_base)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        tf
        or_msgs
        sensor_msgs
        )

catkin_package()

# 通讯节点
add_executable(${PROJECT_NAME}_node
        serial_communication_node.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/hardware/serial_device.cpp)

target_include_directories(${PROJECT_NAME}_node
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME}_node
        PUBLIC
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        glog)

add_dependencies(${PROJECT_NAME}_node or_msgs_generate_messages)




