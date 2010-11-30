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

#ifndef LASER_JOINT_PROCESSOR_LASER_JOINT_PROCESSOR_H_
#define LASER_JOINT_PROCESSOR_LASER_JOINT_PROCESSOR_H_

#include <laser_joint_processor/joint_imager.h>
#include <laser_joint_processor/joint_image_interpolator.h>
#include <settlerlib/sorted_deque.h>
#include <joint_states_settler/joint_states_deflater.h>
#include <sensor_msgs/JointState.h>
#include <calibration_msgs/DenseLaserSnapshot.h>
#include <calibration_msgs/CalibrationPattern.h>
#include <calibration_msgs/JointStateCalibrationPattern.h>


namespace laser_joint_processor
{

class LaserJointProcessor
{
public:

  LaserJointProcessor();

  void setCacheSize(unsigned int max_size);
  void setJointNames(const std::vector<std::string>& joint_names);

  bool addJointState(const sensor_msgs::JointStateConstPtr& joint_state);

  bool isSnapshotEarly(const calibration_msgs::DenseLaserSnapshot& snapshot) const;
  bool isSnapshotLate( const calibration_msgs::DenseLaserSnapshot& snapshot) const;

  ros::Time getEarliestJointStateTime() const;
  ros::Time getLatestJointStateTime() const;

  bool processLaserData( const calibration_msgs::DenseLaserSnapshot& snapshot,
                         const calibration_msgs::CalibrationPattern& features,
                         calibration_msgs::JointStateCalibrationPattern& result,
                         const ros::Duration& max_interp = ros::Duration(.25));

private:
  settlerlib::SortedDeque<joint_states_settler::DeflatedJointStates> joint_state_cache_;
  joint_states_settler::JointStatesDeflater deflater_;
  JointImager imager_;
  JointImageInterpolator interp_;
  std::vector<std::string> joint_names_;
};

}

#endif

