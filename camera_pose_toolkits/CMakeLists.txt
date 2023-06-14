# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(camera_pose_toolkits)

find_package(Boost REQUIRED COMPONENTS regex thread)

find_package(catkin REQUIRED COMPONENTS
  camera_pose_calibration
  dynamic_reconfigure
  geometry_msgs
  kdl_parser
  message_generation
  robot_state_publisher
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)


#uncomment if you have defined services
add_service_files(
  FILES
  Switch.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS
    CATKIN_DEPENDS
      camera_pose_calibration
      dynamic_reconfigure
      geometry_msgs
      kdl_parser
      message_runtime
      robot_state_publisher
      roscpp
      rospy
      sensor_msgs
      std_msgs
      tf
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(transform_finder_node src/transform_finder.cpp)
target_link_libraries(transform_finder_node ${catkin_LIBRARIES})
add_dependencies(transform_finder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(camera_pose_static_transform_tf_broadcaster_node src/camera_pose_static_transform_tf_broadcaster.cpp)
target_link_libraries(camera_pose_static_transform_tf_broadcaster_node ${catkin_LIBRARIES})
add_dependencies(camera_pose_static_transform_tf_broadcaster_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(urdf_writer_node src/urdf_writer.cpp)
target_link_libraries(urdf_writer_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(urdf_writer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(camera_dispatcher_node src/camera_dispatcher.cpp)
target_link_libraries(camera_dispatcher_node ${catkin_LIBRARIES})
add_dependencies(camera_dispatcher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(transform_playback_node src/transform_playback.cpp)
target_link_libraries(transform_playback_node ${catkin_LIBRARIES})
add_dependencies(transform_playback_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

