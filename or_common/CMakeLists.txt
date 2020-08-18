project(or_common)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)

find_package(ProtoBuf REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  )

#这样以来，别的包就可以调用这个包的头文件了
catkin_package(
  INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}
)
