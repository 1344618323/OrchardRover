project(a_star_planner)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

rrts_protobuf_generate_cpp(
        ${CMAKE_CURRENT_SOURCE_DIR}/proto
        AStarPlannerConfigProtoSrc
        AStarPlannerConfigProtoHds
        ${ProtoFiles}
)

add_library(${PROJECT_NAME}
        SHARED
        ${AStarPlannerConfigProtoSrc}
        ${AStarPlannerConfigProtoHds}
        a_star_planner.cpp
        )

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        or_costmap
        )

add_library(planning::global_planner::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
