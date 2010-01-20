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

#include "ros/ros.h"

#include <dense_laser_assembler/dense_laser_assembler.h>

// Messages
#include "sensor_msgs/LaserScan.h"
#include "calibration_msgs/DenseLaserSnapshot.h"
#include "pr2_msgs/LaserScannerSignal.h"

using namespace dense_laser_assembler ;

/**
 * \brief Builds a DenseLaserSnapshot message from laser scans collected in a specified time interval
 */
class DenseLaserSnapshotter
{

public:

  DenseLaserSnapshotter()
  {
    assembler_.setCacheSize(40*20);
    prev_signal_.header.stamp = ros::Time(0, 0);
    signal_sub_ = n_.subscribe("laser_tilt_controller/laser_scanner_signal", 2, &DenseLaserSnapshotter::scannerSignalCallback, this);
    scan_sub_ = n_.subscribe("scan", 1, &DenseLaserSnapshotter::scanCallback, this);
    snapshot_pub_ = n_.advertise<calibration_msgs::DenseLaserSnapshot> ("dense_laser_snapshot", 1);
    first_time_ = true;
  }

  ~DenseLaserSnapshotter()
  {

  }

  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan)
  {
    assembler_.add(scan);
  }

  void scannerSignalCallback(const pr2_msgs::LaserScannerSignalConstPtr& cur_signal)
  {
    ROS_DEBUG("Got Scanner Signal") ;

    if (first_time_)
    {
      prev_signal_ = *cur_signal ;
      first_time_ = false ;
    }
    else
    {
      if (cur_signal->signal == 1)
      {
        ROS_DEBUG("About to make request");

        ros::Time start = prev_signal_.header.stamp;
        ros::Time end = cur_signal->header.stamp;

        calibration_msgs::DenseLaserSnapshot snapshot;

        assembler_.assembleSnapshot(start, end, snapshot);

        ROS_DEBUG("header.stamp: %f", snapshot.header.stamp.toSec());
        ROS_DEBUG("header.frame_id: %s", snapshot.header.frame_id.c_str());
        ROS_DEBUG("ranges.size()=%u", snapshot.ranges.size());
        ROS_DEBUG("intensities.size()=%u", snapshot.intensities.size());
        ROS_DEBUG("scan_start.size()=%u", snapshot.scan_start.size());
        snapshot_pub_.publish(snapshot);
      }
      else
        ROS_DEBUG("Not making request");
      prev_signal_ = *cur_signal;
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher snapshot_pub_;
  ros::Subscriber signal_sub_;
  ros::Subscriber scan_sub_;
  DenseLaserAssembler assembler_;
  pr2_msgs::LaserScannerSignal prev_signal_;

  bool first_time_ ;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dense_laser_snapshotter") ;
  DenseLaserSnapshotter snapshotter ;
  ros::spin() ;

  return 0;
}
