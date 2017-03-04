cmake_minimum_required(VERSION 2.8.3)
project(snoopy_setup)

set(PCL_DIR /usr/share/pcl-1.7)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  PCL REQUIRED
  pcl_ros
  pcl_conversions
  actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
)

add_service_files(
  FILES
  explore.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(kinematics src/kinematics.cpp)
target_link_libraries(kinematics ${catkin_LIBRARIES})
add_dependencies(kinematics snoopy_setup)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
add_dependencies(tf_broadcaster snoopy_setup)

add_executable(snoopy_explores src/snoopy_explores.cpp)
target_link_libraries(snoopy_explores ${catkin_LIBRARIES})
add_dependencies(snoopy_explores snoopy_setup)

add_executable(laser_tf_listener src/laser_tf_listener.cpp)
target_link_libraries(laser_tf_listener ${catkin_LIBRARIES})
add_dependencies(laser_tf_listener snoopy_setup)

add_executable(camera_tf_listener src/camera_tf_listener.cpp)
target_link_libraries(camera_tf_listener ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(camera_tf_listener snoopy_setup)

add_executable(uarm_tf_listener src/uarm_tf_listener.cpp)
target_link_libraries(uarm_tf_listener ${catkin_LIBRARIES})
add_dependencies(uarm_tf_listener snoopy_setup)

add_executable(brain src/brain.cpp)
target_link_libraries(brain ${catkin_LIBRARIES})
add_dependencies(brain snoopy_setup)

add_executable(planB_brain src/planB_brain.cpp)
target_link_libraries(planB_brain ${catkin_LIBRARIES})
add_dependencies(planB_brain snoopy_setup)

add_executable(brainFuck src/brainFuck.cpp)
target_link_libraries(brainFuck ${catkin_LIBRARIES})
add_dependencies(brainFuck snoopy_setup)

#add_executable(speak_test src/speak_test.cpp)
#target_link_libraries(speak_test ${catkin_LIBRARIES})
#add_dependencies(speak_test snoopy_setup)
