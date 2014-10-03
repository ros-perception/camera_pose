# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(kinect_depth_calibration)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS rospy roslib 
  cv_bridge geometry_msgs sensor_msgs stereo_msgs tf_conversions
  tf )


#uncomment if you have defined services
add_service_files(
  FILES
  GetCheckerboardCenter.srv  GetCheckerboardPose.srv
)

catkin_python_setup()

generate_messages(
    DEPENDENCIES geometry_msgs std_msgs
)


# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS rospy roslib opencv2 cv_bridge geometry_msgs sensor_msgs stereo_msgs tf_conversions tf kdl
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
