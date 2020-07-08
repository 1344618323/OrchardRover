project(global_planner)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

add_subdirectory(a_star_planner)

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(
        ${CMAKE_CURRENT_SOURCE_DIR}/proto
        GlobalPlannerProtoSrc
        GlobalPlannerProtoHds
        ${ProtoFiles})

#要注意EXTERNAL_INCLUDE_DIRS是变量，用的时候是${EXTERNAL_INCLUDE_DIRS}
#只有类似于or_costmap这样的共享库，或是global_planner_node.cpp这样的文件，才不用加“${}”
set(EXTERNAL_INCLUDE_DIRS
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/proto
        )

set(EXTERNAL_LIBS
        planning::global_planner::a_star_planner
        or_costmap
        ${catkin_LIBRARIES}
        )

# 全局规划节点
add_executable(${PROJECT_NAME}_node
        ${GlobalPlannerProtoSrc}
        ${GlobalPlannerProtoHds}
        global_planner_node.cpp
        )

target_include_directories(
        ${PROJECT_NAME}_node
        PUBLIC
        ${EXTERNAL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
        PUBLIC
        ${EXTERNAL_LIBS}
        )

add_dependencies(${PROJECT_NAME}_node
        or_msgs_generate_messages)

# 全局规划测试
add_executable(${PROJECT_NAME}_test
        global_planner_test.cpp)
target_include_directories(
        ${PROJECT_NAME}_test
        PRIVATE
        ${EXTERNAL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_test
        PRIVATE
        ${EXTERNAL_LIBS}
        )
add_dependencies(${PROJECT_NAME}_test
        or_msgs_generate_messages)