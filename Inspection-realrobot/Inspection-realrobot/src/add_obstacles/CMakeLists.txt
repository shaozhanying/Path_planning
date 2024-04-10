cmake_minimum_required(VERSION 3.0.2)
project(add_obstacles)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  pcl_ros
  sensor_msgs
)

catkin_package(

  CATKIN_DEPENDS roscpp rospy roslib std_msgs nav_msgs geometry_msgs

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(add_obstacles src/add_obstacles.cpp)

target_link_libraries(add_obstacles ${catkin_LIBRARIES})
