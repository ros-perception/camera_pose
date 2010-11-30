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

//! \author Vijay Pradeep / vpradeep@willowgarage.com

#ifndef LASER_JOINT_PROJECTOR_LASER_JOINT_PROJECTOR_H_
#define LASER_JOINT_PROJECTOR_LASER_JOINT_PROJECTOR_H_

#include <map>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point32.h>

#include <boost/shared_ptr.hpp>

namespace laser_joint_projector
{

class LaserJointProjector
{
public:
  LaserJointProjector();
  void configure(const KDL::Tree& tree, const std::string& root, const std::string& tip);
  geometry_msgs::Point32 project(const std::map<std::string, double>& joint_map);
  sensor_msgs::PointCloud project(const std::vector<sensor_msgs::JointState>& joint_state_vec,
                                  const ros::Duration& time_shift);
private:
  KDL::Chain chain_;
  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> solver_;
};

}

#endif

