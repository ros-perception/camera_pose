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

#include <cstdio>
#include <laser_joint_processor/laser_joint_processor_node.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <boost/thread.hpp>

using namespace laser_joint_processor;
using namespace std;

LaserJointProcessorNode::LaserJointProcessorNode() :
  snapshot_sub_(nh_, "snapshot", 1),
  features_sub_(nh_, "features", 1),
  sync_(snapshot_sub_, features_sub_, 1)
{
  pub_ = nh_.advertise<calibration_msgs::JointStateCalibrationPattern>("joint_features", 1);


  sync_connection_ = sync_.registerCallback( boost::bind( &LaserJointProcessorNode::syncCallback, this, _1, _2));
}

LaserJointProcessorNode::~LaserJointProcessorNode()
{
  sync_connection_.disconnect();
}

void LaserJointProcessorNode::configure(const vector<string>& joint_names, unsigned int max_cache_size)
{
  processor_.setJointNames(joint_names);
  processor_.setCacheSize(max_cache_size);
}

void LaserJointProcessorNode::syncCallback(const calibration_msgs::DenseLaserSnapshotConstPtr& snapshot,
                                           const calibration_msgs::CalibrationPatternConstPtr& features)
{
  if (features->success == 0)
  {
    ROS_DEBUG("Didn't find checkerboard. Exiting");
    return;
  }

  boost::mutex::scoped_lock lock(mutex_);

  if (queued_snapshot_)
    ROS_WARN("Got a snapshot when a queued snapshot has still not been processed. Throwing away older snapshot");

  // We got new data, so we're simply going to throw away the old stuff
  queued_snapshot_.reset();
  queued_features_.reset();

  if (processor_.isSnapshotLate(*snapshot))
  {
    ROS_WARN("Snapshot is too old to be handled by joint state cache. Discarding snapshot");
    return;
  }
  else if (processor_.isSnapshotEarly(*snapshot))
  {
    ROS_DEBUG("Don't have joint_state info for this snapshot yet. Going to queue this snapshot");
    queued_snapshot_ = snapshot;
    queued_features_ = features;
    return;
  }
  ROS_DEBUG("In syncCallback. About to process pair");
  processPair(snapshot, features);
}

void LaserJointProcessorNode::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
  boost::mutex::scoped_lock lock(mutex_);

  //printf("J");
  //fflush(stdout);

  processor_.addJointState(joint_state);

  // We might have to processed the queued data, since we just got a new joint_state message
  if (queued_snapshot_)
  {
    if (processor_.isSnapshotLate(*queued_snapshot_))
    {
      ROS_WARN("Queued snapshot is too old to be handled by joint state cache. Discarding queued snapshot");
      queued_snapshot_.reset();
      queued_features_.reset();
      return;
    }
    else if (processor_.isSnapshotEarly(*queued_snapshot_))
    {
      ROS_DEBUG("Don't have joint_state info for this queued snapshot yet. Going to keep this snapshot queued");
      return;
    }
    else
    {
      ROS_DEBUG("In jointStateCallback. About to process pair");
      processPair(queued_snapshot_, queued_features_);
      queued_snapshot_.reset();
      queued_features_.reset();
    }
  }
}

void LaserJointProcessorNode::processPair(const calibration_msgs::DenseLaserSnapshotConstPtr& snapshot,
                                          const calibration_msgs::CalibrationPatternConstPtr& features)
{
  calibration_msgs::JointStateCalibrationPattern result;
  bool success;
  success = processor_.processLaserData(*snapshot, *features, result);
  if (!success)
  {
    ROS_ERROR("Error while calling LaserJointProcessor::processLaserData");
    return;
  }

  pub_.publish(result);

}

void getParamConfig(ros::NodeHandle& nh, vector<string>& joint_names)
{
  joint_names.clear();

  bool found_joint = true;
  unsigned int joint_count = 0;
  char param_buf[1024] ;
  while(found_joint)
  {
    sprintf(param_buf, "joint_name_%02u", joint_count) ;
    std::string param_name = param_buf ;
    std::string cur_joint_name ;
    found_joint = nh.getParam(param_name, cur_joint_name) ;
    if (found_joint)
    {
      ROS_INFO("[%s] -> [%s]", param_name.c_str(), cur_joint_name.c_str());
      joint_names.push_back(cur_joint_name);
      joint_count++ ;
    }
  }

  if (joint_names.size() == 0)
    ROS_INFO("Found no joints on param server");
}

using namespace laser_joint_processor;



void spinFunc(ros::CallbackQueue* queue)
{
  ros::SingleThreadedSpinner spinner;
  spinner.spin(queue);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_joint_processor");
  ros::NodeHandle private_nh("~");

  LaserJointProcessorNode processor;

  // Put joint states callback on its own queue
  ros::NodeHandle n;
  ros::Subscriber joint_state_sub;

  ros::CallbackQueue queue;
  ros::SubscribeOptions ops;
  ops.init<sensor_msgs::JointState>("joint_states", 2000, boost::bind(&LaserJointProcessorNode::jointStateCallback, &processor, _1));
  ops.transport_hints = ros::TransportHints();
  ops.callback_queue = &queue;
  joint_state_sub = n.subscribe(ops);

  vector<string> joint_names;
  getParamConfig(private_nh, joint_names);
  processor.configure(joint_names, 2000);

  boost::function<void()> func = boost::bind(&spinFunc, (ros::CallbackQueue*) &queue);

  boost::thread jointStateSpinner(func);
  ros::spin();
  printf("Waiting for thread to join");
  jointStateSpinner.join();
  printf("Done joining");
  return 0;
}








