cmake_minimum_required(VERSION 3.0.2)
project(inspection_waypoints)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(

  CATKIN_DEPENDS roscpp rospy roslib std_msgs nav_msgs geometry_msgs

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(inspection_waypoints src/inspection_waypoints.cpp)

target_link_libraries(inspection_waypoints ${catkin_LIBRARIES})
