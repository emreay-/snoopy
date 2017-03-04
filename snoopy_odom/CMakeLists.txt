cmake_minimum_required(VERSION 2.8.3)
project(snoopy_odom)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  message_generation
)

add_service_files(
  FILES
  update_odom.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()
add_executable(dead_reck src/dead_reck.cpp)
target_link_libraries(dead_reck ${catkin_LIBRARIES})
add_dependencies(dead_reck snoopy_odom_gencpp)
