cmake_minimum_required(VERSION 2.8.3)
project(snoopy_vision)


set(openCV_DIR /usr/share/OpenCV)
set(PCL_DIR /usr/share/pcl-1.7)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  cv_bridge
  image_transport
  openCV
  PCL REQUIRED
  geometry_msgs
)

add_service_files(
  FILES
  identify.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES snoopy_vision
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
#target_link_libraries(my_awesome_library ${OpenCV_LIBRARIES})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable(color_detector src/color_detector.cpp)
#target_link_libraries(color_detector ${catkin_LIBRARIES})
#add_dependencies(color_detector snoopy_vision)

#add_executable(color_depth_detector src/color_depth_detector.cpp)
#target_link_libraries(color_depth_detector ${catkin_LIBRARIES})
#add_dependencies(color_depth_detector snoopy_vision)

#add_executable(color_pcl src/color_pcl.cpp)
#target_link_libraries(color_pcl ${PCL_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(color_pcl snoopy_vision)

#add_executable(preprocess_v2 src/preprocess_v2.cpp)
#target_link_libraries(preprocess_v2 ${PCL_LIBRARIES} ${catkin_LIBRARIES})
#add_dependencies(preprocess_v2 snoopy_vision)

add_executable(snoopy_qr src/snoopy_qr.cpp)
target_link_libraries(snoopy_qr ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_dependencies(snoopy_qr snoopy_vision)

add_executable(preprocess src/preprocess.cpp)
target_link_libraries(preprocess ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(preprocess snoopy_vision)

add_executable(ident_obj src/ident_obj.cpp)
target_link_libraries(ident_obj ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(ident_obj snoopy_vision)

add_executable(object_detect src/object_detect.cpp)
target_link_libraries(object_detect ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(object_detect snoopy_vision)
