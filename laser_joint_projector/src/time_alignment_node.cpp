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

#include <ros/ros.h>
#include <laser_joint_projector/laser_joint_projector.h>
#include <kdl_parser/kdl_parser.hpp>
#include <calibration_msgs/JointStateCalibrationPattern.h>
#include <std_msgs/Float64.h>
#include <boost/thread/mutex.hpp>

using namespace laser_joint_projector;
using namespace std;

class TimeAlignmentNode
{
public:
  TimeAlignmentNode()
  {
    cloud_num_ = 0;

    std::string robot_desc_string;
    nh_.param("robot_description", robot_desc_string, string());
    if (!kdl_parser::treeFromString(robot_desc_string, tree_))
       ROS_FATAL("Failed to construct kdl tree");

    pub0_ = nh_.advertise<sensor_msgs::PointCloud>("cb_cloud_0", 1);
    pub1_ = nh_.advertise<sensor_msgs::PointCloud>("cb_cloud_1", 1);
    sub_features_ = nh_.subscribe("joint_state_features", 1, &TimeAlignmentNode::featuresCallback, this);
    sub_offset_   = nh_.subscribe("keyboard_float", 1, &TimeAlignmentNode::offsetCallback, this);

    ros::NodeHandle private_nh_("~");

    private_nh_.param("root", root_, std::string("root_link"));
    private_nh_.param("tip",  tip_,  std::string("tip_link"));

    projector_.configure(tree_, root_, tip_);
  }

private:
  ros::NodeHandle nh_;
  unsigned int cloud_num_;
  KDL::Tree tree_;
  ros::Publisher pub0_;
  ros::Publisher pub1_;
  ros::Subscriber sub_features_;
  ros::Subscriber sub_offset_;
  LaserJointProjector projector_;

  double offset_;
  std::string root_;
  std::string tip_;

  boost::mutex mutex_;

  calibration_msgs::JointStateCalibrationPatternConstPtr features_0_;
  calibration_msgs::JointStateCalibrationPatternConstPtr features_1_;

  void featuresCallback(const calibration_msgs::JointStateCalibrationPatternConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (cloud_num_ == 0)
      features_0_ = msg;
    else if (cloud_num_ == 1)
      features_1_ = msg;
    else
      ROS_FATAL("Cloud num is [%u]", cloud_num_);

    // Toggle the cloud #
    cloud_num_ = 1 - cloud_num_;

    publish();
  }

  void offsetCallback(const std_msgs::Float64ConstPtr& offset)
  {
    boost::mutex::scoped_lock lock(mutex_);
    offset_ = offset->data;
    publish();
  }

  void publish()
  {
    ros::Duration offset(offset_);

    sensor_msgs::PointCloud cloud_0;
    sensor_msgs::PointCloud cloud_1;

    if (features_0_)
    {
      cloud_0 = projector_.project(features_0_->joint_points, offset);
      cloud_0.header.stamp = features_0_->header.stamp;
    }

    if (features_1_)
    {
      cloud_1 = projector_.project(features_1_->joint_points, offset);
      cloud_1.header.stamp = features_1_->header.stamp;
    }

    cloud_0.header.frame_id = root_;
    cloud_1.header.frame_id = root_;

    pub0_.publish(cloud_0);
    pub1_.publish(cloud_1);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "time_alignment");

  TimeAlignmentNode alignment_node;

  ros::spin();

  return 0;
}
