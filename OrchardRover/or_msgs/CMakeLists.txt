project(or_msgs)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
    nav_msgs
    geometry_msgs
    actionlib
    )

add_message_files(
    DIRECTORY msg
    FILES
    TrunkObsMsg.msg
    TrunkObsMsgXY.msg
    TwistAccel.msg
)

add_action_files(
    DIRECTORY action
    FILES
    LocalPlanner.action
    GlobalPlanner.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
