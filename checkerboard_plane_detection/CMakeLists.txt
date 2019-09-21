cmake_minimum_required(VERSION 2.8.3)
project(checkerboard_plane_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  velodyne_pointcloud
)

find_package(PCL REQUIRED)

catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS
        DEPENDS PCL
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node
        src/checkerboard_plane_detection_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)
