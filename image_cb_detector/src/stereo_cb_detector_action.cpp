/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <image_cb_detector/image_cb_detector_old.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <image_cb_detector/ConfigAction.h>
#include <calibration_msgs/Interval.h>
#include <stereo_msgs/DisparityImage.h>

#include <cv_bridge/CvBridge.h>

using namespace image_cb_detector;

class StereoCbDetectorAction
{
public:
  StereoCbDetectorAction() : as_("cb_detector_config", false), it_(nh_),
                             image_synchronizer_(ApproxTimeSync(30), sub_image_, sub_cam_info_, sub_disparity_)
  {
    as_.registerGoalCallback( boost::bind(&StereoCbDetectorAction::goalCallback, this) );
    as_.registerPreemptCallback( boost::bind(&StereoCbDetectorAction::preemptCallback, this) );

    pub_ = nh_.advertise<calibration_msgs::CalibrationPattern>("features",1);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("cb_pose",1);

    image_synchronizer_.registerCallback(boost::bind(&StereoCbDetectorAction::syncCb, this, _1, _2, _3));

    sub_image_.subscribe(nh_, "image", 10);
    sub_cam_info_.subscribe(nh_, "camera_info", 10);
    sub_disparity_.subscribe(nh_, "disparity", 10);

    as_.start();
  }

  void goalCallback()
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    ROS_INFO("In StereoCbDetector Goal Callback");

    // Stop the previously running goal (if it exists)
    if (as_.isActive())
      as_.setPreempted();

    // Get the new goal from the action server
    image_cb_detector::ConfigGoalConstPtr goal = as_.acceptNewGoal();
    assert(goal);

    // Try to reconfigure the settler object
    bool success = detector_.configure(*goal);

    // Detect possible failure
    if (!success)
      as_.setAborted();
  }

  void preemptCallback()
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    // Don't need to do any cleanup. Immeadiately turn it off
    as_.setPreempted();
  }

  void syncCb(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cam_info, const stereo_msgs::DisparityImage::ConstPtr& disparity)
  {
    boost::mutex::scoped_lock lock(run_mutex_);

    if (as_.isActive())
    {
      calibration_msgs::CalibrationPattern features;
      bool success;
      success = detector_.detect(image, cam_info, features);

      if (!success)
      {
        ROS_ERROR("Error trying to detect checkerboard, not going to publish CalibrationPattern");
        return;
      }

      // Figure out the depth reading
      sensor_msgs::ImageConstPtr disp_image = actionlib::share_member(disparity, disparity->image);
      cv::Mat disparity = bridge_.imgMsgToCv(disp_image, "passthrough");  // "32FC1"

      for (unsigned int i=0; i < features.image_points.size(); i++)
      {
        double cur_d = disparity.at<float>( floor(features.image_points[i].y + 0.499), floor(features.image_points[i].x + 0.499));

        /*if ( cur_d == 0.0 )
        {
          ROS_DEBUG("Got NaNs");
          features.success = 0;
        }
        */
        features.image_points[i].d = cur_d;
      }
      //printf("\n");

      /* const unsigned int N = features.image_points.size();

      // Allocate Maps
      cv::Mat_<double> map_x(N,1);
      cv::Mat_<double> map_y(N,1);

      // Set up maps
      for (unsigned int i=0; i<N; i++)
      {
        map_x.at<double>(i,0) = features.image_points[i].x;
        map_y.at<double>(i,0) = features.image_points[i].y;
      }

      // Allocate Destination Image
      cv::Mat_<double> dest_disparity(N,1);

      // Perform the OpenCV interpolation
      // @TODO: Check if map_x and map_y are backwards
      cv::remap(disparity, dest_disparity, map_x, map_y, cv::INTER_LINEAR);

      for (unsigned int i=0; i<N; i++)
      {
        features.image_points[i].d = dest_disparity.at<double>(i,0);
      }
      */
      pose_pub_.publish(features.object_pose);
      pub_.publish(features);
    }
  }

private:
  boost::mutex run_mutex_;
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<image_cb_detector::ConfigAction> as_;
  ImageCbDetectorOld detector_;

  ros::Publisher pose_pub_;
  ros::Publisher pub_;
  image_transport::ImageTransport it_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, stereo_msgs::DisparityImage > ApproxTimeSync;

  message_filters::Subscriber<sensor_msgs::Image> sub_image_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> sub_cam_info_;
  message_filters::Subscriber<stereo_msgs::DisparityImage> sub_disparity_;
  message_filters::Synchronizer<ApproxTimeSync> image_synchronizer_;

  sensor_msgs::CvBridge bridge_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_states_settler_action");

  ros::NodeHandle n;

  StereoCbDetectorAction detector_action;

  ros::spin();

  return 0;
}
