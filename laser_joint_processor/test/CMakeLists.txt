set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********

rosbuild_add_gtest(test/joint_imager_unittest joint_imager_unittest.cpp)
target_link_libraries(test/joint_imager_unittest laser_joint_processor)

rosbuild_add_gtest(test/joint_image_interpolator_unittest joint_image_interpolator_unittest.cpp)
target_link_libraries(test/joint_image_interpolator_unittest laser_joint_processor)

rosbuild_add_gtest(test/interp_snapshot_unittest interp_snapshot_unittest.cpp)
target_link_libraries(test/interp_snapshot_unittest laser_joint_processor)

rosbuild_add_gtest(test/laser_joint_processor_unittest laser_joint_processor_unittest.cpp)
target_link_libraries(test/laser_joint_processor_unittest laser_joint_processor)
