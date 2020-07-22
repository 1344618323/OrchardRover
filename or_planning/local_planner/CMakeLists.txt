project(local_planner)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)


add_subdirectory(timed_elastic_band)

set(EXTERNAL_INCLUDE_DIRS
        ${EIGEN3_INCLUDE_DIR}
        ${SUITESPARSE_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        )

set(EXTERNAL_LIBS
        ${SUITESPARSE_LIBRARIES}
        ${G2O_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        or_costmap
        )

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
rrts_protobuf_generate_cpp(${CMAKE_CURRENT_SOURCE_DIR}/proto
        LocalPlanProtoSrc
        LocalPlanProtoHds
        ${ProtoFiles}
        )

aux_source_directory(src/. SRC_LIST)

#local planner node
add_executable(${PROJECT_NAME}_node
        ${LocalPlanProtoSrc}
        ${SRC_LIST}
        local_planner_node.cpp
        )

target_include_directories(${PROJECT_NAME}_node
        PUBLIC
        ${EXTERNAL_INCLUDE_DIRS}
        )

target_link_libraries(${PROJECT_NAME}_node
        PUBLIC
        planning::timed_elastic_band
        ${EXTERNAL_LIBS}
        )
add_dependencies(${PROJECT_NAME}_node
        or_msgs_generate_messages)

#local palnner test
add_executable(teb_test
        ${LocalPlanProtoSrc}
        ${SRC_LIST}
        teb_test.cpp
        )

target_include_directories(teb_test
        PUBLIC
        ${EXTERNAL_INCLUDE_DIRS}
        )

target_link_libraries(teb_test
        PUBLIC
        planning::timed_elastic_band
        ${EXTERNAL_LIBS}
        )
add_dependencies(teb_test
        or_msgs_generate_messages)

#vel converter
add_executable(vel_converter
       vel_converter.cpp
       )

target_include_directories(vel_converter
       PUBLIC
       ${EXTERNAL_INCLUDE_DIRS}
       )

target_link_libraries(vel_converter
        PUBLIC
       ${EXTERNAL_LIBS}
       )
add_dependencies(vel_converter
       or_msgs_generate_messages)
