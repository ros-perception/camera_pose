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

#ifndef LASER_JOINT_PROCESSOR_JOINT_IMAGER_H_
#define LASER_JOINT_PROCESSOR_JOINT_IMAGER_H_

#include <settlerlib/sorted_deque.h>
#include <joint_states_settler/deflated_joint_states.h>
#include <calibration_msgs/DenseLaserSnapshot.h>
#include <opencv/cv.h>

namespace laser_joint_processor
{

class JointImager
{
public:
  JointImager();
  ~JointImager();

  bool update(const calibration_msgs::DenseLaserSnapshot& snapshot,
              const settlerlib::SortedDeque<joint_states_settler::DeflatedJointStates>& cache,
              const ros::Duration& max_interp = ros::Duration(.25));

  void displayImage(unsigned int i);
  void writeImage(unsigned int i, const std::string& filename);
  IplImage* getJointImage(unsigned int index) const;

protected:
  std::vector< IplImage* > images;

  bool interpPosition(const ros::Time& target,
                      const settlerlib::SortedDeque<joint_states_settler::DeflatedJointStates>& cache,
                      std::vector<double>& result);

  bool computeVelocity(const ros::Time& start, const ros::Time& end,
                       const settlerlib::SortedDeque<joint_states_settler::DeflatedJointStates>& cache,
                       std::vector<double>& result);

  void allocateImages(unsigned int height, unsigned int width, unsigned int channels);

};

}


#endif

