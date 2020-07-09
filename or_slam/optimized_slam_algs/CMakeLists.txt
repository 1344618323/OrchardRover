cmake_minimum_required(VERSION 3.1)
project(optimized_slam_algs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(Ceres REQUIRED)

add_library(${PROJECT_NAME}
        optimized_slam.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/transform/rigid_transform.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/cost_function/spa_cost_function_2d.cpp
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${EIGEN3_LIBRARIES}
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES}
        glog
        )