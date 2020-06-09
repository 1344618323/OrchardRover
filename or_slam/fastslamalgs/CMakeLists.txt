cmake_minimum_required(VERSION 3.1)

project(fastslamalgs)

set(CMAKE_CXX_STANDARD 14)

add_library(${PROJECT_NAME}
        fastslam.cpp
        fastslam.h
        fastslam_config.h
        fastslam_localization.cpp
        fastslam_localization.h
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensor_odom.h
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/sensor_odom.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter.h
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter_gaussian_pdf.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter_gaussian_pdf.h
        ${CMAKE_CURRENT_SOURCE_DIR}/particle_filter/particle_filter_sample.h
        )

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        ${EIGEN3_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${EIGEN3_LIBRARIES}
        ${catkin_LIBRARIES}
        # ${GLOG_LIBRARY}
        )
