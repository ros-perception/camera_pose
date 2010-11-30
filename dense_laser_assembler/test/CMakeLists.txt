set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

# ********** Tests **********

rosbuild_add_gtest(test/dense_laser_assembler_unittest dense_laser_assembler_unittest.cpp)
target_link_libraries(test/dense_laser_assembler_unittest dense_laser_assembler)