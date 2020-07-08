project(or_planning)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        pcl_conversions
        pcl_ros
        nav_msgs
        geometry_msgs
        interactive_markers
        actionlib
        or_common
        or_msgs
        or_costmap
        )

find_package(G2O REQUIRED)
find_package(SUITESPARSE REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(ProtoBuf REQUIRED)

catkin_package()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/global_planner)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/local_planner)


