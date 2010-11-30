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

using namespace laser_joint_projector;
using namespace std;

void featuresCallback(ros::Publisher* pub,
                      LaserJointProjector* projector,
                      const calibration_msgs::JointStateCalibrationPatternConstPtr& msg)
{
  sensor_msgs::PointCloud cloud = projector->project(msg->joint_points, ros::Duration());
  cloud.header.stamp = msg->header.stamp;
  cloud.header.frame_id = "torso_lift_link"; //! \todo param
  pub->publish(cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "projector");
  ros::NodeHandle n;

  KDL::Tree my_tree;
  std::string robot_desc_string;
  if (!n.getParam("robot_description", robot_desc_string))
    ROS_FATAL("Couldn't get a robot_description from the param server");

  if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
  {
     ROS_ERROR("Failed to construct kdl tree");
     return false;
  }

//  KDL::Tree my_tree;
//  string filename= "/u/vpradeep/ros/pkgs/ros-pkg/stacks/robot_calibration/laser_joint_projector/pr2.urdf";
//  if (!kdl_parser::treeFromFile(filename, my_tree))
//  {
//     ROS_ERROR("Failed to construct kdl tree");
//     return -1;
//  }

  LaserJointProjector projector;
  projector.configure(my_tree, "torso_lift_link", "laser_tilt_link");

  // Output
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>("checkerboard_points", 1);

  // Input
  boost::function<void (const calibration_msgs::JointStateCalibrationPatternConstPtr&)> cb
      = boost::bind(&featuresCallback, &pub, &projector, _1);

  ros::Subscriber sub = n.subscribe(std::string("joint_state_features"), 1, cb);

  ros::spin();

  return 0;
}
