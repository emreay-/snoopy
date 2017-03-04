cmake_minimum_required(VERSION 2.8.3)
project(snoopy_map)
##set(DCMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  nav_msgs
  geometry_msgs
  tf
)

add_service_files(
  FILES
  ogm_get.srv
  ogm_update.srv 
  ogm_get_cell.srv
  ogm_info.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ogm_server src/ogm_server.cpp)
target_link_libraries(ogm_server ${catkin_LIBRARIES})
add_dependencies(ogm_server snoopy_map_gencpp)

add_executable(ogm_client src/ogm_client.cpp)
target_link_libraries(ogm_client ${catkin_LIBRARIES})
add_dependencies(ogm_client snoopy_map_gencpp)

add_executable(update_obstacle src/update_obstacle.cpp)
target_link_libraries(update_obstacle ${catkin_LIBRARIES})
add_dependencies(update_obstacle snoopy_map_gencpp)


