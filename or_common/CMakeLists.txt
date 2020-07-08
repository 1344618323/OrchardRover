project(or_common)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
)
