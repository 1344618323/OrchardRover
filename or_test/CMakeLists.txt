cmake_minimum_required(VERSION 3.1)
project(or_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        cv_bridge
        image_transport
        sensor_msgs
        rospy
        or_msgs
        message_filters
        nav_msgs
        tf
        rosbag
        )

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

catkin_package()

## TestPublish
add_executable(testpublish
        src/testpublish.cpp
        )
target_include_directories(testpublish
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        )
target_link_libraries(testpublish
        ${catkin_LIBRARIES}
        )
add_dependencies(testpublish or_msgs_generate_messages)


## TestListener
add_executable(testlistener
        src/testlistener.cpp
        )
target_include_directories(testlistener
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        )
target_link_libraries(testlistener
        ${catkin_LIBRARIES}
        )
add_dependencies(testlistener or_msgs_generate_messages)

## timesynchronizer
add_executable(timesynchronizer
        src/timesynchronizer.cpp
        )
target_include_directories(timesynchronizer
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        )
target_link_libraries(timesynchronizer
        ${catkin_LIBRARIES}
        )
add_dependencies(timesynchronizer or_msgs_generate_messages)

## turtle_tf_broadcaster
add_executable(turtle_tf_broadcaster
        src/turtle_tf_broadcaster.cpp
        )
target_include_directories(turtle_tf_broadcaster
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        )
target_link_libraries(turtle_tf_broadcaster
        ${catkin_LIBRARIES}
        )

## turtle_tf_bturtle_tf_listenerroadcaster
add_executable(turtle_tf_listener
        src/turtle_tf_listener.cpp
        )
target_include_directories(turtle_tf_listener
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        )
target_link_libraries(turtle_tf_listener
        ${catkin_LIBRARIES}
        )
add_dependencies(turtle_tf_listener or_msgs_generate_messages)

## add_executable(robot_vision
add_executable(robot_vision
        src/robot_vision.cpp
        )

target_include_directories(robot_vision
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        )

target_link_libraries(robot_vision
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

add_executable(test_bag
        src/test_bag.cpp
        )

target_include_directories(test_bag
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        )

target_link_libraries(test_bag
        ${catkin_LIBRARIES}
        )

catkin_install_python(PROGRAMS
        scripts/testpylistener.py
        scripts/testpytalker.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
        )
