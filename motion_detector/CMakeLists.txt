cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 14)
project(motion_detector)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  sensor_msgs
  normal_msg
  line_msg
  cv_bridge
  pcl_ros
)

find_package( PCL REQUIRED)
find_package( OpenCV REQUIRED )

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(motion_detector_node src/motion_detector_node.cpp)
target_link_libraries(motion_detector_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

