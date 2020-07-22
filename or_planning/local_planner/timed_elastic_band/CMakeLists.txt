project(timed_elastic_band)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

aux_source_directory(src/. SRC_LIST)

# 尽量避免直接包 ${CMAKE_CURRENT_SOURCE_DIR}吧
# 写成         ${CMAKE_CURRENT_SOURCE_DIR}/include
#             ${CMAKE_CURRENT_SOURCE_DIR}/proto
# 更有指向性一点
set(EXTERNAL_INCLUDE_DIRS
        ${EIGEN3_INCLUDE_DIR}
        ${SUITESPARSE_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/proto
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
        )

set(EXTERNAL_LIBS
        ${SUITESPARSE_LIBRARIES}
        ${G2O_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        )

file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")

rrts_protobuf_generate_cpp(
        ${CMAKE_CURRENT_SOURCE_DIR}/proto
        timed_elastic_bandProtoSrc
        timed_elastic_bandProtoHds
        ${ProtoFiles}
)

add_library(${PROJECT_NAME}
        SHARED
        ${timed_elastic_bandProtoSrc}
        ${SRC_LIST}
        )

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${EXTERNAL_INCLUDE_DIRS}
        )

target_link_libraries(${PROJECT_NAME}
        PUBLIC
        ${EXTERNAL_LIBS}
        )

add_library(planning::timed_elastic_band ALIAS timed_elastic_band)
