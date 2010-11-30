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

#include <cstdio>
#include <laser_joint_projector/laser_joint_projector.h>
#include <calibration_msgs/JointStateCalibrationPattern.h>

using namespace laser_joint_projector;
using namespace std;

LaserJointProjector::LaserJointProjector()
{

}

void LaserJointProjector::configure(const KDL::Tree& tree,
                                    const std::string& root, const std::string& tip)
{
  bool success;
  success = tree.getChain(root, tip, chain_);
  if (!success)
    ROS_ERROR("Error extracting chain from [%s] to [%s]\n", root.c_str(), tip.c_str());

  KDL::Segment angle_segment("laser_angle_segment",
                             KDL::Joint("laser_angle_joint", KDL::Joint::RotZ));
  KDL::Segment range_segment("laser_range_segment",
                             KDL::Joint("laser_range_joint", KDL::Joint::TransX));

  chain_.addSegment(angle_segment);
  chain_.addSegment(range_segment);

  for (unsigned int i=0; i < chain_.segments.size(); i++)
  {
    printf("%2u) %s -> %s\n", i, chain_.segments[i].getName().c_str(),
                                  chain_.segments[i].getJoint().getName().c_str());
  }

  solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
}

sensor_msgs::PointCloud LaserJointProjector::project(const vector<sensor_msgs::JointState>& joint_state_vec, const ros::Duration& time_shift)
{
  sensor_msgs::PointCloud cloud;
  cloud.points.clear();

  for (unsigned int i=0; i<joint_state_vec.size(); i++)
  {
    map<string, double> joint_map;
    for (unsigned int j=0; j<joint_state_vec[i].name.size(); j++)
    {
      joint_map.insert(make_pair(joint_state_vec[i].name[j],
                                 joint_state_vec[i].position[j] +  joint_state_vec[i].velocity[j]*time_shift.toSec()));
    }
    geometry_msgs::Point32 pt = project(joint_map);
    cloud.points.push_back(pt);
  }

  return cloud;
}


geometry_msgs::Point32 LaserJointProjector::project(const map<string, double>& joint_map)
{
  KDL::JntArray joint_pos_array(chain_.getNrOfJoints());

  // Populate the JntArray, by looking into map, based on joint names in chain
  unsigned int cur_joint_num=0;
  for (unsigned int i=0; i<chain_.getNrOfSegments(); i++)
  {
    if (chain_.getSegment(i).getJoint().getType() != KDL::Joint::None)
    {
      string joint_name = chain_.getSegment(i).getJoint().getName();
      map<string, double>::const_iterator it = joint_map.find(joint_name);

      if (it == joint_map.end())
      {
        ROS_ERROR("Couldn't find joint [%s] in map", joint_name.c_str());
        joint_pos_array(cur_joint_num, 0) = 0.0;
      }
      else
      {
        joint_pos_array(cur_joint_num, 0) = (*it).second;
      }
      cur_joint_num++;
    }
  }

  KDL::Frame out_frame;
  solver_->JntToCart(joint_pos_array, out_frame);

  geometry_msgs::Point32 pt;
  pt.x = out_frame.p.data[0];
  pt.y = out_frame.p.data[1];
  pt.z = out_frame.p.data[2];

  return pt;
}
