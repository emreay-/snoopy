cmake_minimum_required(VERSION 2.8.3)
project(snoopy_teleop)
find_package(catkin REQUIRED COMPONENTS
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

add_executable(snoopy_teleop src/snoopy_teleop.cpp)
target_link_libraries(snoopy_teleop ${catkin_LIBRARIES})
add_dependencies(snoopy_teleop snoopy_teleop)


