cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 14)
project(synchronizer)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  pcl_ros
  roscpp
  sensor_msgs
)

find_package( PCL REQUIRED)
find_package( OpenCV REQUIRED )
catkin_package(
 INCLUDE_DIRS
 CATKIN_DEPENDS
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  DEPENDS PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/synchronizer_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
)

