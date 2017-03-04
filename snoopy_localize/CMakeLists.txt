cmake_minimum_required(VERSION 2.8.3)
project(snoopy_localize)
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
)

add_service_files(
  FILES
  get_pose.srv
  fuse.srv
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

#add_executable(particleFilter src/particleFilter.cpp)
#target_link_libraries(particleFilter ${catkin_LIBRARIES})
#add_dependencies(particleFilter snoopy_localize_gencpp)

#add_executable(particleFilter2 src/particleFilter2.cpp)
#target_link_libraries(particleFilter2 ${catkin_LIBRARIES})
#add_dependencies(particleFilter2 snoopy_localize_gencpp)

#add_executable(particleFilter3 src/particleFilter3.cpp)
#target_link_libraries(particleFilter3 ${catkin_LIBRARIES})
#add_dependencies(particleFilter3 snoopy_localize_gencpp)

add_executable(particleFilter4 src/particleFilter4.cpp)
target_link_libraries(particleFilter4 ${catkin_LIBRARIES})
add_dependencies(particleFilter4 snoopy_localize_gencpp)

add_executable(pose_fusion src/pose_fusion.cpp)
target_link_libraries(pose_fusion ${catkin_LIBRARIES})
add_dependencies(pose_fusion snoopy_localize_gencpp)
