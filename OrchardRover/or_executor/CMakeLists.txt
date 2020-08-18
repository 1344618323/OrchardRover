project(or_executor)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  or_msgs
  )

catkin_package()

add_executable(${PROJECT_NAME}_node
        executor_node.cpp
        chassis_executor.cpp
        )

target_include_directories(${PROJECT_NAME}_node
        PUBLIC
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        )

target_link_libraries(${PROJECT_NAME}_node
        PUBLIC
        ${catkin_LIBRARIES}
        )

add_dependencies(${PROJECT_NAME}_node
        or_msgs_generate_messages)
