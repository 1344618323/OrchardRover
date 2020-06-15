cmake_minimum_required(VERSION 3.1)
project(or_slam)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        nav_msgs
        message_filters
        sensor_msgs
        or_msgs
        )

catkin_package()

add_subdirectory(optimized_slam_algs)
add_subdirectory(fastslamalgs)

add_executable(${PROJECT_NAME}_node
        slam_node.cpp
        )

target_include_directories(${PROJECT_NAME}_node
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        )

target_link_libraries(${PROJECT_NAME}_node
        fastslamalgs
        optimized_slam_algs
        ${catkin_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        glog
        )

add_dependencies(${PROJECT_NAME}_node or_msgs_generate_messages)