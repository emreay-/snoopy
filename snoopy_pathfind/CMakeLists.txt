  cmake_minimum_required(VERSION 2.8.3)
project(snoopy_pathfind)
#set(DCMAKE_BUILD_TYPE Debug)
set(openCV_DIR /usr/share/OpenCV)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  cv_bridge
  openCV
)

add_service_files(
	FILES
	pathfind.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(astar src/astar.cpp)
target_link_libraries(astar ${catkin_LIBRARIES})
add_dependencies(astar snoopy_pathfind_gencpp)

add_executable(astar_client src/astar_client.cpp)
target_link_libraries(astar_client ${catkin_LIBRARIES})
add_dependencies(astar_client snoopy_pathfind_gencpp)
