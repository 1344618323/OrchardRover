cmake_minimum_required(VERSION 3.1)
project(hk_camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        cv_bridge
        image_transport
        camera_info_manager
)

catkin_package()

find_package(OpenCV REQUIRED)

#添加库文件搜索路径，一定要在 add_executable要使用该库文件执行文件 之前写，否则无效
link_directories(
        /opt/MVS/lib/64/
)

add_executable(${PROJECT_NAME}_node
        hk_camera_node.cpp
        hk_camera.cpp
        )

target_include_directories(${PROJECT_NAME}_node 
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        /opt/MVS/include/
        )

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        MvCameraControl
        )
