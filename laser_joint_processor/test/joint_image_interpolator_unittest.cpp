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


float& pixel(IplImage* image, int row, int col, int chan)
{
  return *(((float*)(image->imageData + row*image->widthStep))+image->nChannels*col + chan);
}

class JointImageInterpolator_EasyTests : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    source_image_ = cvCreateImage( cvSize(3,3), IPL_DEPTH_32F, 2);
    pixel(source_image_, 0, 0, 0) = 10;
    pixel(source_image_, 0, 1, 0) = 20;
    pixel(source_image_, 0, 2, 0) = 30;
    pixel(source_image_, 1, 0, 0) = 40;
    pixel(source_image_, 1, 1, 0) = 50;
    pixel(source_image_, 1, 2, 0) = 60;
    pixel(source_image_, 2, 0, 0) = 70;
    pixel(source_image_, 2, 1, 0) = 80;
    pixel(source_image_, 2, 2, 0) = 90;

    pixel(source_image_, 0, 0, 1) = 1;
    pixel(source_image_, 0, 1, 1) = 2;
    pixel(source_image_, 0, 2, 1) = 3;
    pixel(source_image_, 1, 0, 1) = 4;
    pixel(source_image_, 1, 1, 1) = 5;
    pixel(source_image_, 1, 2, 1) = 6;
    pixel(source_image_, 2, 0, 1) = 7;
    pixel(source_image_, 2, 1, 1) = 8;
    pixel(source_image_, 2, 2, 1) = 9;

    // Pixel Values:
    //  1  2  3
    //  4  5  6
    //  7  8  9

    image_points_.resize(3);
    image_points_[0].x = 0.0;
    image_points_[0].y = 0.0;

    image_points_[1].x = 0.5;
    image_points_[1].y = 0.5;

    image_points_[2].x = 1;
    image_points_[2].y = 0.75;

    bool success;
    success = interp_.interp(image_points_, source_image_, positions_, velocities_);
    ASSERT_TRUE(success);
  }

  virtual void TearDown()
  {
    cvReleaseImage(&source_image_);
  }

  IplImage* source_image_;
  vector<calibration_msgs::ImagePoint> image_points_;
  JointImageInterpolator interp_;
  vector<float> positions_;
  vector<float> velocities_;
};

TEST_F(JointImageInterpolator_EasyTests, positionTests)
{
  ASSERT_EQ(positions_.size(), (unsigned int) 3);
  EXPECT_NEAR(positions_[0], 10.0, eps);
  EXPECT_NEAR(positions_[1], 30.0, eps);
  EXPECT_NEAR(positions_[2], 42.5, eps);
}

TEST_F(JointImageInterpolator_EasyTests, velocityTests)
{
  ASSERT_EQ(velocities_.size(), (unsigned int) 3);
  EXPECT_NEAR(velocities_[0], 1.0, eps);
  EXPECT_NEAR(velocities_[1], 3.0, eps);
  EXPECT_NEAR(velocities_[2], 4.25, eps);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
