cmake_minimum_required(VERSION 3.1)
project(or_lasercamcal)
SET(CMAKE_BUILD_TYPE "Release") # Debug
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        image_transport
        roscpp
        rosbag
        sensor_msgs
        geometry_msgs
        tf
        cv_bridge
        message_generation
        nodelet
        std_msgs
        cmake_modules
        )

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

catkin_package()


file(GLOB_RECURSE c_files
        ${CMAKE_CURRENT_SOURCE_DIR}/ethz_apriltag/*.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/camera_models/*.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
        )

add_library(laser_cam_cal
        ${c_files}
        )

target_include_directories(laser_cam_cal
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/camera_models
        ${CMAKE_CURRENT_SOURCE_DIR}/ethz_apriltag
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${Eigen_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        )

target_link_libraries(laser_cam_cal
        ${Eigen_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_executable(kalibra_detect_node
        ${CMAKE_CURRENT_SOURCE_DIR}/main/kalibratag_detector_node.cpp
        )
target_link_libraries(kalibra_detect_node
        laser_cam_cal
        )

add_executable(lasercamcal_node
        ${CMAKE_CURRENT_SOURCE_DIR}/main/calibr_offline.cpp
        )
target_link_libraries(lasercamcal_node
        laser_cam_cal
        )


add_executable(simulation_lasercamcal_node
        ${CMAKE_CURRENT_SOURCE_DIR}/main/calibr_simulation.cpp)
target_link_libraries(simulation_lasercamcal_node
        laser_cam_cal
        )

add_executable(showscan_node
        ${CMAKE_CURRENT_SOURCE_DIR}/main/showscan_node.cpp)

target_link_libraries(showscan_node
		laser_cam_cal
        )
