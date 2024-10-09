cmake_minimum_required(VERSION 2.8.3)
project(ouster_rosbag_parser)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  rosbag
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ouster_rosbag_parser_node src/ouster_rosbag_parser_node.cpp)
target_link_libraries(ouster_rosbag_parser_node ${catkin_LIBRARIES})
