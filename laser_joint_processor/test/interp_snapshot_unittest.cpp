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
#include <laser_joint_processor/joint_image_interpolator.h>

using namespace std;
using namespace laser_joint_processor;

static const float eps = 1e-6;

class InterpSnapshot_EasyTests : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    snapshot_.angle_min = 10.0;
    snapshot_.angle_increment = 1.0;
    snapshot_.readings_per_scan = 4;
    snapshot_.num_scans = 3;
    snapshot_.ranges.resize(12);
    for (unsigned int i=0; i<snapshot_.ranges.size(); i++)
      snapshot_.ranges[i] = i*100;

    points_.resize(7);

    // Boundaries
    points_[0] = Point(0,0);
    points_[1] = Point(3,0);
    points_[2] = Point(0,2);
    points_[3] = Point(3,2);

    points_[4] = Point(2.5, .5);
    points_[5] = Point(2,    1);
    points_[6] = Point(1.25,  1.5);

    bool result;
    result = interpSnapshot(points_, snapshot_, angles_, ranges_);
    ASSERT_TRUE(result);
  }

  calibration_msgs::ImagePoint Point(float x, float y)
  {
    calibration_msgs::ImagePoint point;
    point.x = x;
    point.y = y;
    return point;
  }

  std::vector <calibration_msgs::ImagePoint> points_;
  vector<float> angles_;
  vector<float> ranges_;
  calibration_msgs::DenseLaserSnapshot snapshot_;
};

TEST_F(InterpSnapshot_EasyTests, angles)
{
  EXPECT_NEAR(angles_[0], 10.0, eps);
  EXPECT_NEAR(angles_[1], 13.0, eps);
  EXPECT_NEAR(angles_[2], 10.0, eps);
  EXPECT_NEAR(angles_[3], 13.0, eps);
  EXPECT_NEAR(angles_[4], 12.5, eps);
  EXPECT_NEAR(angles_[5], 12.0, eps);
  EXPECT_NEAR(angles_[6], 11.25,eps);
}


/*
 *   0  100  200  300
 * 400  500  600  700
 * 800  900 1000 1100
 */

TEST_F(InterpSnapshot_EasyTests, ranges)
{
  EXPECT_NEAR(ranges_[0],    0.0, eps);
  EXPECT_NEAR(ranges_[1],  300.0, eps);
  EXPECT_NEAR(ranges_[2],  800.0, eps);
  EXPECT_NEAR(ranges_[3], 1100.0, eps);
  EXPECT_NEAR(ranges_[4],  450.0, eps);
  EXPECT_NEAR(ranges_[5],  600.0, eps);
  EXPECT_NEAR(ranges_[6],  725.0, eps);
}

TEST_F(InterpSnapshot_EasyTests, out_of_bounds)
{
  points_[6] = Point(-1.0, 0);
  EXPECT_FALSE(interpSnapshot(points_, snapshot_, angles_, ranges_));

  points_[6] = Point(0, -1.0);
  EXPECT_FALSE(interpSnapshot(points_, snapshot_, angles_, ranges_));

  points_[6] = Point(3.1, 1.9);
  EXPECT_FALSE(interpSnapshot(points_, snapshot_, angles_, ranges_));

  points_[6] = Point(2.9, 2.1);
  EXPECT_FALSE(interpSnapshot(points_, snapshot_, angles_, ranges_));

  // Control Test
  points_[6] = Point(1, 1);
  EXPECT_TRUE(interpSnapshot(points_, snapshot_, angles_, ranges_));
}



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
