cmake_minimum_required(VERSION 2.8.3)
project(snoopy_navigate)
##set(DCMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  nav_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
)
add_action_files(
   DIRECTORY action
   FILES 
   navigate.action
)
generate_messages(
 DEPENDENCIES 
 actionlib_msgs 
 std_msgs
 geometry_msgs
 nav_msgs
)


catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

#add_executable(navigate src/navigate.cpp)
#target_link_libraries(navigate ${catkin_LIBRARIES})
#add_dependencies(navigate snoopy_navigate_gencpp)

#add_executable(nav_test src/nav_test.cpp)
#target_link_libraries(nav_test ${catkin_LIBRARIES})
#add_dependencies(nav_test snoopy_navigate_gencpp)

#add_executable(nav_control_test src/nav_control_test.cpp)
#target_link_libraries(nav_control_test ${catkin_LIBRARIES})
#add_dependencies(nav_control_test snoopy_navigate_gencpp ${snoopy_navigate_EXPORTED_TARGETS})

add_executable(navigate_action_server src/navigate_action_server.cpp)
target_link_libraries(navigate_action_server ${catkin_LIBRARIES})
add_dependencies(navigate_action_server snoopy_navigate_gencpp ${snoopy_navigate_EXPORTED_TARGETS})

add_executable(navigate_action_client src/navigate_action_client.cpp)
target_link_libraries(navigate_action_client ${catkin_LIBRARIES})
add_dependencies(navigate_action_client snoopy_navigate_gencpp ${snoopy_navigate_EXPORTED_TARGETS})

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
target_link_libraries(obstacle_avoidance ${catkin_LIBRARIES})
add_dependencies(obstacle_avoidance snoopy_navigate_gencpp ${snoopy_navigate_EXPORTED_TARGETS})
