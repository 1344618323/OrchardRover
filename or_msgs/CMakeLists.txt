project(or_msgs)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
    )

add_message_files(
    DIRECTORY msg
    FILES
    TrunkObsMsg.msg
    TrunkObsMsgXY.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
