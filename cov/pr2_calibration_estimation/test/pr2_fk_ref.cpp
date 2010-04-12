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
#include <kdl_parser/kdl_parser.hpp>
#include <pr2_calibration_estimation/FkTest.h>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

using namespace std;
using namespace pr2_calibration_estimation;

bool fk(KDL::Tree* tree, FkTest::Request& req, FkTest::Response& resp)
{
  KDL::Chain my_chain;
  bool success;
  success = tree->getChain(req.root, req.tip, my_chain);

  if (!success)
  {
    ROS_ERROR("Error extracting chain from [%s] to [%s]\n", req.root.c_str(), req.tip.c_str());
    return false;
  }

  KDL::ChainFkSolverPos_recursive solver(my_chain);

  KDL::JntArray joints(my_chain.getNrOfJoints());
  for (unsigned int i=0; i<my_chain.getNrOfJoints(); i++)
    joints(i) = req.joint_positions[i];

  KDL::Frame out_frame;
  if (solver.JntToCart(joints, out_frame))
  {
    ROS_ERROR("Error running KDL solver");
    return false;
  }

  resp.pos.resize(3);

  // Copy over translation vector
  resp.pos[0] = out_frame.p.data[0];
  resp.pos[1] = out_frame.p.data[1];
  resp.pos[2] = out_frame.p.data[2];

  // Copy over rotation matrix
  resp.rot.resize(9);
  for (unsigned int i=0; i<9; i++)
    resp.rot[i] = out_frame.M.data[i];

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fk_reference");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  KDL::Tree my_tree;
  string robot_desc_string;
  if (!n.getParam("robot_description", robot_desc_string))
    ROS_FATAL("Couldn't get a robot_description from the param server");

  if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
  {
     ROS_ERROR("Failed to construct kdl tree");
     return false;
  }

  boost::function< bool(FkTest::Request&,  FkTest::Response&) > func = boost::bind(fk, &my_tree, _1, _2) ;
  ros::ServiceServer fk_service = n.advertiseService("fk", func);

  ros::spin();

  return 0;
}
