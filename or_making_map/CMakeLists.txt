cmake_minimum_required(VERSION 3.1)
project(making_maps)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(OpenCV)

add_executable(making_map
        making_map.cpp
        )
target_include_directories(making_map
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        )
target_link_libraries(making_map
        PUBLIC
        ${OpenCV_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        )

add_executable(making_map_from_gmapping
        making_map_from_gmapping.cpp
        )
target_include_directories(making_map_from_gmapping
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        )
target_link_libraries(making_map_from_gmapping
        PUBLIC
        ${OpenCV_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        )
