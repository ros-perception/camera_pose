/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <gtest/gtest.h>
#include <laser_joint_processor/laser_joint_processor.h>

using namespace std;
using namespace laser_joint_processor;

static const float eps = 1e-6;

class LaserJointProcessor_EasyTests : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    bool success;

    // Set up joints of interest
    vector<string> names;
    names.push_back("JointA");
    names.push_back("JointB");
    processor_.setJointNames(names);
    processor_.setCacheSize(1000);

    // Populate the cache
    for (unsigned int i=0; i<20; i++)
    {
      sensor_msgs::JointStatePtr joint_state(new sensor_msgs::JointState);
      joint_state->header.stamp = ros::Time(100+i, 0);
      joint_state->name.push_back("JointA");
      joint_state->name.push_back("JointB");
      joint_state->name.push_back("JointC");
      joint_state->position.push_back(i*1.0);
      joint_state->position.push_back(i*10.0);
      joint_state->position.push_back(i*20.0);
      joint_state->velocity.resize(3);
      joint_state->effort.resize(3);

      success = processor_.addJointState(joint_state);
      ASSERT_TRUE(success);
    }

    // Build The Snapshot
    snapshot_.angle_min = 10.0;
    snapshot_.angle_increment = 1.0;
    snapshot_.time_increment  = .5;
    snapshot_.readings_per_scan = 4;
    snapshot_.num_scans = 3;
    snapshot_.ranges.resize(12);
    for (unsigned int i=0; i<snapshot_.ranges.size(); i++)
      snapshot_.ranges[i] = i*100;
    snapshot_.scan_start.resize(snapshot_.num_scans);
    snapshot_.scan_start[0] = ros::Time(105);
    snapshot_.scan_start[1] = ros::Time(110);
    snapshot_.scan_start[2] = ros::Time(115.1);

    // Build Feature Vector
    features_.image_points.resize(P);

    // Boundaries
    features_.image_points[0] = Point(0,0);
    features_.image_points[1] = Point(3,0);
    features_.image_points[2] = Point(0,2);
    features_.image_points[3] = Point(3,2);

    features_.image_points[4] = Point(2.5, .5);
    features_.image_points[5] = Point(2,    1);
    features_.image_points[6] = Point(1.25,  1.5);
  }

  void testProcessing()
  {
    bool success;
    // Run the processing step
    success = processor_.processLaserData(snapshot_, features_, result_, ros::Duration(2,0));
    EXPECT_TRUE(success);
  }

  calibration_msgs::ImagePoint Point(float x, float y)
  {
    calibration_msgs::ImagePoint point;
    point.x = x;
    point.y = y;
    return point;
  }

  static const unsigned int P = 7;
  calibration_msgs::JointStateCalibrationPattern result_;
  calibration_msgs::CalibrationPattern features_;
  calibration_msgs::DenseLaserSnapshot snapshot_;
  LaserJointProcessor processor_;
};

TEST_F(LaserJointProcessor_EasyTests, earlyTest)
{
  EXPECT_FALSE(processor_.isSnapshotEarly(snapshot_));
  EXPECT_FALSE(processor_.isSnapshotLate(snapshot_));
  snapshot_.scan_start[2] = ros::Time(300);
  EXPECT_TRUE(processor_.isSnapshotEarly(snapshot_));
  EXPECT_FALSE(processor_.isSnapshotLate(snapshot_));
}

TEST_F(LaserJointProcessor_EasyTests, lateTest)
{
  EXPECT_FALSE(processor_.isSnapshotLate(snapshot_));
  EXPECT_FALSE(processor_.isSnapshotEarly(snapshot_));
  snapshot_.scan_start[0] = ros::Time(10);
  EXPECT_TRUE(processor_.isSnapshotLate(snapshot_));
  EXPECT_FALSE(processor_.isSnapshotEarly(snapshot_));
}

/*
 * Joint 1:
 *  5.00   5.50   6.00   6.50
 * 10.00  10.50  11.00  11.50
 * 15.10  15.60  16.10  16.60
 *
 * Joint 2:
 *  50.00   55.00   60.00   65.00
 * 100.00  105.00  110.00  115.00
 * 151.00  156.00  161.00  166.00
 */

TEST_F(LaserJointProcessor_EasyTests, jointTest)
{
  testProcessing();

  ASSERT_EQ(result_.joint_points.size(), (unsigned int) P);
  for (unsigned int i=0; i<result_.joint_points.size(); i++)
  {
    ASSERT_EQ(result_.joint_points[i].name.size(), (unsigned int) 4);
    ASSERT_EQ(result_.joint_points[i].position.size(), (unsigned int) 4);
    EXPECT_EQ(result_.joint_points[i].name[0], "JointA");
    EXPECT_EQ(result_.joint_points[i].name[1], "JointB");
  }

  // point #0 = (0,0)
  // t=105
  EXPECT_NEAR(result_.joint_points[0].position[0],  5.0, eps);
  EXPECT_NEAR(result_.joint_points[0].position[1], 50.0, eps);

  // point #1 = (3,0)
  // t=1.5
  EXPECT_NEAR(result_.joint_points[1].position[0],  6.5, eps);
  EXPECT_NEAR(result_.joint_points[1].position[1],   65, eps);

  // point #2 = (0,2)
  // t=115.1
  EXPECT_NEAR(result_.joint_points[2].position[0],  15.1, eps);
  EXPECT_NEAR(result_.joint_points[2].position[1],   151, eps);

  // point #3 = (3,2)
  // t=116.6
  EXPECT_NEAR(result_.joint_points[3].position[0],  16.6, eps);
  EXPECT_NEAR(result_.joint_points[3].position[1],   166, eps);

  // point #4 = (2.5, .5)
  EXPECT_NEAR(result_.joint_points[4].position[0],  8.75, eps);
  EXPECT_NEAR(result_.joint_points[4].position[1],  87.5, eps);

  // point #5 = (2, 1)
  EXPECT_NEAR(result_.joint_points[5].position[0],   11, eps);
  EXPECT_NEAR(result_.joint_points[5].position[1],  110, eps);

  // point #6 = (1.25, 1.5)
  EXPECT_NEAR(result_.joint_points[6].position[0],  13.175, eps);
  EXPECT_NEAR(result_.joint_points[6].position[1],  131.75, eps);
}

TEST_F(LaserJointProcessor_EasyTests, anglesTest)
{
  testProcessing();

  ASSERT_EQ(result_.joint_points.size(), (unsigned int) P);
  for (unsigned int i=0; i<P; i++)
  {
    ASSERT_EQ(result_.joint_points[i].name.size(), (unsigned int) 4);
    ASSERT_EQ(result_.joint_points[i].position.size(), (unsigned int) 4);
    EXPECT_EQ(result_.joint_points[i].name[2], "laser_angle_joint");
  }

  EXPECT_NEAR(result_.joint_points[0].position[2], 10.0, eps);
  EXPECT_NEAR(result_.joint_points[1].position[2], 13.0, eps);
  EXPECT_NEAR(result_.joint_points[2].position[2], 10.0, eps);
  EXPECT_NEAR(result_.joint_points[3].position[2], 13.0, eps);
  EXPECT_NEAR(result_.joint_points[4].position[2], 12.5, eps);
  EXPECT_NEAR(result_.joint_points[5].position[2], 12.0, eps);
  EXPECT_NEAR(result_.joint_points[6].position[2], 11.25, eps);
}

TEST_F(LaserJointProcessor_EasyTests, rangesTest)
{
  testProcessing();

  ASSERT_EQ(result_.joint_points.size(), (unsigned int) P);
  for (unsigned int i=0; i<P; i++)
  {
    ASSERT_EQ(result_.joint_points[i].name.size(), (unsigned int) 4);
    ASSERT_EQ(result_.joint_points[i].position.size(), (unsigned int) 4);
    EXPECT_EQ(result_.joint_points[i].name[3], "laser_range_joint");
  }

  EXPECT_NEAR(result_.joint_points[0].position[3],    0.0, eps);
  EXPECT_NEAR(result_.joint_points[1].position[3],  300.0, eps);
  EXPECT_NEAR(result_.joint_points[2].position[3],  800.0, eps);
  EXPECT_NEAR(result_.joint_points[3].position[3], 1100.0, eps);
  EXPECT_NEAR(result_.joint_points[4].position[3],  450.0, eps);
  EXPECT_NEAR(result_.joint_points[5].position[3],  600.0, eps);
  EXPECT_NEAR(result_.joint_points[6].position[3],  725.0, eps);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
