# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(camera_pose_toolkits)

find_package(Boost REQUIRED COMPONENTS regex thread)

find_package(catkin REQUIRED COMPONENTS std_msgs rospy roscpp tf geometry_msgs
  sensor_msgs camera_pose_calibration kdl_parser robot_state_publisher
  dynamic_reconfigure rosbag)


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
    DEPENDS std_msgs rospy roscpp tf geometry_msgs sensor_msgs camera_pose_calibration kdl kdl_parser robot_state_publisher dynamic_reconfigure
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

add_executable(transform_finder_node src/transform_finder.cpp)
target_link_libraries(transform_finder_node ${catkin_LIBRARIES})

add_executable(camera_pose_static_transform_tf_broadcaster_node src/camera_pose_static_transform_tf_broadcaster.cpp)
target_link_libraries(camera_pose_static_transform_tf_broadcaster_node ${catkin_LIBRARIES})

add_executable(urdf_writer_node src/urdf_writer.cpp)
target_link_libraries(urdf_writer_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(camera_dispatcher_node src/camera_dispatcher.cpp)
target_link_libraries(camera_dispatcher_node ${catkin_LIBRARIES})

add_executable(transform_playback_node src/transform_playback.cpp)
target_link_libraries(transform_playback_node ${catkin_LIBRARIES})

