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
#include <laser_joint_processor/joint_imager.h>

using namespace std;
using namespace laser_joint_processor;
using joint_states_settler::DeflatedJointStates;

static const float eps = 1e-6;

typedef settlerlib::SortedDeque<joint_states_settler::DeflatedJointStates> DeflatedDeque;

void populateCache(DeflatedDeque& cache, const ros::Time& start, const unsigned int N)
{
  for (unsigned int i=0; i<N; i++)
  {
    joint_states_settler::DeflatedJointStates deflated;
    deflated.header.stamp = start + ros::Duration(i,0);
    deflated.channels_.resize(2);
    deflated.channels_[0] =  10*i;
    deflated.channels_[1] = 100*i;

    cache.add(deflated);
  }
}


class JointImager_EasyTests : public ::testing::Test
{
protected:
  virtual void SetUp()
  {
    cache_.setMaxSize(1000);
    populateCache(cache_, ros::Time(0,0), 100);

    snapshot_.time_increment = 1;
    snapshot_.readings_per_scan = 3;
    snapshot_.num_scans = 4;
    snapshot_.scan_start.resize(4);
    snapshot_.scan_start[0] = ros::Time(.5);
    snapshot_.scan_start[1] = ros::Time(10);
    snapshot_.scan_start[2] = ros::Time(35);
    snapshot_.scan_start[3] = ros::Time(50.5);

    bool success;
    success =imager_.update(snapshot_, cache_, ros::Duration(2,0));
    ASSERT_TRUE(success);

    image0_ = imager_.getJointImage(0);
    image1_ = imager_.getJointImage(1);
  }

  DeflatedDeque cache_;
  calibration_msgs::DenseLaserSnapshot snapshot_;
  JointImager imager_;

  IplImage* image0_;
  IplImage* image1_;
};

float imageAt(IplImage* image, int row, int col, int channel)
{
  return *(((float*)(image->imageData + row*image->widthStep)) + image->nChannels*col + channel);
}

TEST_F(JointImager_EasyTests, positions)
{
  EXPECT_NEAR(imageAt(image0_, 0, 0, 0),   5, eps);
  EXPECT_NEAR(imageAt(image0_, 1, 0, 0), 100, eps);
  EXPECT_NEAR(imageAt(image0_, 2, 0, 0), 350, eps);
  EXPECT_NEAR(imageAt(image0_, 0, 2, 0),  25, eps);
  EXPECT_NEAR(imageAt(image0_, 3, 2, 0), 525, eps);

  EXPECT_NEAR(imageAt(image1_, 0, 0, 0),   50, eps);
  EXPECT_NEAR(imageAt(image1_, 3, 2, 0), 5250, eps);

}

TEST_F(JointImager_EasyTests, velocities)
{
  EXPECT_NEAR(imageAt(image0_, 0, 0, 1),  10, eps);
  EXPECT_NEAR(imageAt(image0_, 0, 1, 1),  10, eps);
  EXPECT_NEAR(imageAt(image0_, 0, 2, 1),  10, eps);
  EXPECT_NEAR(imageAt(image0_, 3, 0, 1),  10, eps);
  EXPECT_NEAR(imageAt(image0_, 3, 1, 1),  10, eps);
  EXPECT_NEAR(imageAt(image0_, 3, 2, 1),  10, eps);

  EXPECT_NEAR(imageAt(image1_, 0, 0, 1), 100, eps);
  EXPECT_NEAR(imageAt(image1_, 0, 1, 1), 100, eps);
  EXPECT_NEAR(imageAt(image1_, 0, 2, 1), 100, eps);
  EXPECT_NEAR(imageAt(image1_, 3, 0, 1), 100, eps);
  EXPECT_NEAR(imageAt(image1_, 3, 1, 1), 100, eps);
  EXPECT_NEAR(imageAt(image1_, 3, 2, 1), 100, eps);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
