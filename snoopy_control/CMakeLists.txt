cmake_minimum_required(VERSION 2.8.3)
project(snoopy_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES snoopy_teleop
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pid_controller src/pid_controller.cpp)
target_link_libraries(pid_controller ${catkin_LIBRARIES})
add_dependencies(pid_controller snoopy_control)
