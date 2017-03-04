cmake_minimum_required(VERSION 2.8.3)
project(snoopy_uarm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ObjectPosition.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(uarm src/uarm.cpp)
target_link_libraries(uarm ${catkin_LIBRARIES})
add_dependencies(uarm snoopy_uarm_gencpp)

#add_executable(uarm_pick_server_test src/uarm_pick_server2.cpp)
#target_link_libraries(uarm_pick_server_test ${catkin_LIBRARIES})
#add_dependencies(uarm_pick_server_test snoopy_uarm_gencpp)

