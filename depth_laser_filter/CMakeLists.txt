cmake_minimum_required(VERSION 2.8.3)
project(depth_laser_filter)

set(THIS_PACKAGE_ROS_DEPS sensor_msgs roscpp tf filters message_filters laser_geometry pluginlib angles)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
find_package(Boost REQUIRED COMPONENTS system signals)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


catkin_package(
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS
)

add_executable(max_range_filter_node src/depth_filter_node.cpp)
target_link_libraries(max_range_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(single_readings_filter_node src/single_readings_filter_node.cpp)
target_link_libraries(single_readings_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
