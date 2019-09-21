cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 14)
project(lidar_edge_detector)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  velodyne_pointcloud
  message_filters
  cv_bridge
  tf
  tf_conversions
)

find_package( PCL REQUIRED)
find_package( OpenCV REQUIRED)
find_package( Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
		src/lidar_edge_detector_node.cpp
		src/Velodyne.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${CERES_LIBRARIES}
)

