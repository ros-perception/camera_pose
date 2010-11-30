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

//! \author Vijay Pradeep

#include <laser_joint_processor/laser_joint_processor.h>

using namespace laser_joint_processor;
using namespace std;

LaserJointProcessor::LaserJointProcessor()
{

}

void LaserJointProcessor::setCacheSize(unsigned int max_size)
{
  joint_state_cache_.setMaxSize(max_size);
}

void LaserJointProcessor::setJointNames(const std::vector<std::string>& joint_names)
{
  joint_names_ = joint_names;
  deflater_.setDeflationJointNames(joint_names);
}

bool LaserJointProcessor::addJointState(const sensor_msgs::JointStateConstPtr& joint_state)
{
  joint_states_settler::DeflatedJointState deflated;
  deflater_.deflate(joint_state, deflated);
  //printf("Deflated: %f\n", deflated.channels_[0]);
  joint_state_cache_.add(deflated);

  return true;
}

bool LaserJointProcessor::isSnapshotEarly(const calibration_msgs::DenseLaserSnapshot& snapshot) const
{
  ROS_DEBUG("Checking if snapshot is early:");
  if (joint_state_cache_.size() == 0)
  {
    ROS_DEBUG("  cache is empty, so snapshot is definitely early");
    return true;
  }
  if (snapshot.scan_start.size() == 0)
    ROS_FATAL("Received malformed DenseLaserSnapshot. (scan_start.size() == 0)");
  ros::Time snapshot_end = snapshot.scan_start.back() + ros::Duration(snapshot.time_increment*(snapshot.num_scans-1));

  ros::Time cache_front = joint_state_cache_.front().header.stamp;
  ros::Time cache_back = joint_state_cache_.back().header.stamp;
  ROS_DEBUG("  Newest elem in cache (back):  %u.%u", cache_back.sec, cache_back.nsec);
  ROS_DEBUG("  Oldest elem in cache (front): %u.%u", cache_front.sec, cache_front.nsec);
  ROS_DEBUG("  End of snapshot:      %u.%u", snapshot_end.sec, snapshot_end.nsec);
  ros::Duration diff = snapshot_end - cache_back;
  ROS_DEBUG("  cache_back - snapshot_end: %.2f", diff.toSec());

  return  diff >= ros::Duration(0,0);
}

bool LaserJointProcessor::isSnapshotLate(const calibration_msgs::DenseLaserSnapshot& snapshot) const
{
  ROS_DEBUG("Checking if snapshot is late:");
  if (joint_state_cache_.size() == 0)
  {
    ROS_DEBUG("  cache is empty, so snapshot isn't late yet");
    return false;
  }

  if (snapshot.scan_start.size() == 0)
    ROS_FATAL("Received malformed DenseLaserSnapshot. (scan_start.size() == 0)");
  ros::Time snapshot_start = snapshot.scan_start.front();
  ros::Time cache_front = joint_state_cache_.front().header.stamp;
  ros::Time cache_back = joint_state_cache_.back().header.stamp;

  ros::Duration diff = snapshot_start - cache_front;

  ROS_DEBUG("  Newest elem in cache (back):  %u.%u", cache_back.sec, cache_back.nsec);
  ROS_DEBUG("  Oldest elem in cache (front): %u.%u", cache_front.sec, cache_front.nsec);
  ROS_DEBUG("  Start of snapshot:            %u.%u", snapshot_start.sec, snapshot_start.nsec);
  ROS_DEBUG("  cache_front - snapshot_start: %.2f", diff.toSec());

  return diff <= ros::Duration(0,0);
}

bool LaserJointProcessor::processLaserData( const calibration_msgs::DenseLaserSnapshot& snapshot,
                                            const calibration_msgs::CalibrationPattern& features,
                                            calibration_msgs::JointStateCalibrationPattern& result,
                                            const ros::Duration& max_interp)
{
  // Store # of joints
  const unsigned int num_joints = joint_names_.size();
  // store # of feature points
  const unsigned int P = features.image_points.size();

  result.object_points = features.object_points;
  result.joint_points.clear();
  result.joint_points.resize(P);

  bool success;
  success = imager_.update(snapshot, joint_state_cache_, max_interp);

  //imager_.displayImage(0);
/*  imager_.writeImage(0, "/wg/stor2/vpradeep/ros/pkgs/wg-ros-pkg/calibration_experimental/pr2_calibration_launch/data/image.txt");


  FILE* file;
  file = fopen("/wg/stor2/vpradeep/ros/pkgs/wg-ros-pkg/calibration_experimental/pr2_calibration_launch/data/scan_start.txt", "w");
  for (unsigned int i=0; i<snapshot.scan_start.size(); i++)
  {
    fprintf(file, "%u.%u\n", snapshot.scan_start[i].sec, snapshot.scan_start[i].nsec);
  }
  fclose(file);

  file = fopen("/wg/stor2/vpradeep/ros/pkgs/wg-ros-pkg/calibration_experimental/pr2_calibration_launch/data/joint_cache.txt", "w");
  for (unsigned int i=0; i<joint_state_cache_.size(); i++)
  {
    fprintf(file, "%u.%u  ", joint_state_cache_[i].header.stamp.sec, joint_state_cache_[i].header.stamp.nsec);
    for (unsigned int j=0; j<joint_state_cache_[i].channels_.size(); j++)
    {
      fprintf(file, "%4.3f  ", joint_state_cache_[i].channels_[j]);
    }
    fprintf(file, "\n");
  }
  fclose(file);

  file = fopen("/wg/stor2/vpradeep/ros/pkgs/wg-ros-pkg/calibration_experimental/pr2_calibration_launch/data/features.txt", "w");
  for (unsigned int i=0; i<features.image_points.size(); i++)
  {
    fprintf(file, "%4.3f  %4.3f\n", features.image_points[i].x, features.image_points[i].y);
  }
  fclose(file);
*/
  //printf("\n");
  //imager_.displayImage(1);

  if (!success)
  {
    ROS_ERROR("imager_.update failed");
    return false;
  }

  for (unsigned int j=0; j<num_joints; j++)
  {

    vector<float> positions;
    vector<float> velocities;
    interp_.interp(features.image_points, imager_.getJointImage(j),
                   positions, velocities);

    for (unsigned int i=0; i<P; i++)
    {
      result.joint_points[i].name.push_back(joint_names_[j]);
      result.joint_points[i].position.push_back(positions[i]);
      result.joint_points[i].velocity.push_back(velocities[i]);
      result.joint_points[i].effort.push_back(1234);
    }
  }

  vector<float> laser_angles;
  vector<float> laser_ranges;

  success = interpSnapshot(features.image_points, snapshot, laser_angles, laser_ranges);
  if (!success)
  {
    ROS_ERROR("interpSnapshot failed");
    return false;
  }

  for (unsigned int i=0; i<P; i++)
  {
    result.joint_points[i].name.push_back("laser_angle_joint");
    result.joint_points[i].position.push_back(laser_angles[i]);
    result.joint_points[i].velocity.push_back(0);
    result.joint_points[i].effort.push_back(1234);

    result.joint_points[i].name.push_back("laser_range_joint");
    result.joint_points[i].position.push_back(laser_ranges[i]);
    result.joint_points[i].velocity.push_back(0);
    result.joint_points[i].effort.push_back(1234);
  }

  result.header.stamp = snapshot.header.stamp;

  return true;
}




