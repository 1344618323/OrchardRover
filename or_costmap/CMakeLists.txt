project(or_costmap)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
find_package(Eigen3 REQUIRED)
find_package(ProtoBuf REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        pcl_conversions
        pcl_ros
        laser_geometry
        sensor_msgs
        geometry_msgs
        or_io
        )

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
)


#proto file
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
        CostmapProtoSrcs
        CostmapProtoHeaders
        ${ProtoFiles}
        )

aux_source_directory(src/. SRC_LIST)
list(REMOVE_ITEM SRC_LIST "src/test_costmap.cpp")

# #lib project
add_library(or_costmap
        SHARED
        ${SRC_LIST}
        ${CostmapProtoSrcs}
        ${CostmapProtoHeaders}
        )

target_include_directories(or_costmap
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/proto
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        )

target_link_libraries(or_costmap
        PUBLIC
        ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        )

#test project
add_executable(test_costmap src/test_costmap.cpp)

target_include_directories(test_costmap
        PUBLIC
        )

target_link_libraries(test_costmap
        or_costmap
        )

