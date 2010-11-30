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

#include <sensor_msgs/LaserScan.h>
#include <dense_laser_assembler/dense_laser_assembler.h>

using namespace std;
using namespace dense_laser_assembler;

sensor_msgs::LaserScanConstPtr buildScan(const ros::Time& stamp, double start_angle, double start_range, double start_intensity, const unsigned int N)
{
  sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan);

  scan->header.stamp = stamp;

  scan->angle_increment = 1.0;
  scan->angle_min = start_angle;
  scan->angle_max = start_angle + (N-1)*scan->angle_increment;
  scan->time_increment = 1.0;
  scan->scan_time = 0.0;
  scan->range_min = 0.0;
  scan->range_max = 1000.0;
  scan->ranges.resize(N);
  scan->intensities.resize(N);
  for (unsigned int i=0; i<N; i++)
  {
    scan->ranges[i] = start_range + 1.0*i;
    scan->intensities[i] = start_intensity + 1.0*i;
    //printf("%.3f ", scan->ranges[i]);
  }
  //printf("\n");
  return scan;
}

static const unsigned int NUM_SCANS = 20;
static const unsigned int RAYS_PER_SCAN = 50;
static const double eps = 1e-8;

void addScans1(const ros::Time& start_stamp, double start_angle, DenseLaserAssembler& assembler)
{

  for (unsigned int i=0; i<NUM_SCANS; i++)
  {
    sensor_msgs::LaserScanConstPtr scan = buildScan(start_stamp + ros::Duration(i,0), start_angle, 100*i, 1000*i, RAYS_PER_SCAN);
    assembler.add(scan);
  }
}

TEST(DenseLaserAssembler, easy)
{
  DenseLaserAssembler assembler;
  addScans1(ros::Time(10,0), 0.0, assembler);

  calibration_msgs::DenseLaserSnapshot snapshot;
  bool result;
  result = assembler.assembleSnapshot(ros::Time(9,0), ros::Time(31,0), snapshot);
  ASSERT_TRUE(result);
  ASSERT_EQ(snapshot.ranges.size(),      NUM_SCANS * RAYS_PER_SCAN);
  ASSERT_EQ(snapshot.intensities.size(), NUM_SCANS * RAYS_PER_SCAN);
  ASSERT_EQ(snapshot.readings_per_scan, RAYS_PER_SCAN);
  ASSERT_EQ(snapshot.num_scans, NUM_SCANS);
  ASSERT_EQ(snapshot.scan_start.size(), NUM_SCANS);

  // Check start time
  EXPECT_EQ(snapshot.scan_start[0],  ros::Time(10,0));
  EXPECT_EQ(snapshot.scan_start[5],  ros::Time(15,0));
  EXPECT_EQ(snapshot.scan_start[19], ros::Time(29,0));

  // Check ranges
  EXPECT_NEAR(snapshot.ranges[0],      0, eps);
  EXPECT_NEAR(snapshot.ranges[70],   120, eps);
  EXPECT_NEAR(snapshot.ranges[999], 1949, eps);

  // Check intensities
  EXPECT_NEAR(snapshot.intensities[0],       0, eps);
  EXPECT_NEAR(snapshot.intensities[70],   1020, eps);
  EXPECT_NEAR(snapshot.intensities[999], 19049, eps);
}

TEST(DenseLaserAssembler, mismatchedMetadata)
{
  DenseLaserAssembler assembler;
  addScans1(ros::Time( 10,0), 0.0, assembler);
  addScans1(ros::Time(110,0), 1.0, assembler);

  calibration_msgs::DenseLaserSnapshot snapshot;
  bool result;
  result = assembler.assembleSnapshot(ros::Time(9,0), ros::Time(31,0), snapshot);
  EXPECT_TRUE(result);
  EXPECT_EQ(snapshot.ranges.size(), NUM_SCANS * RAYS_PER_SCAN);

  result = assembler.assembleSnapshot(ros::Time(109,0), ros::Time(131,0), snapshot);
  EXPECT_TRUE(result);
  EXPECT_EQ(snapshot.ranges.size(), NUM_SCANS * RAYS_PER_SCAN);

  result = assembler.assembleSnapshot(ros::Time(9,0), ros::Time(131,0), snapshot);
  EXPECT_FALSE(result);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
