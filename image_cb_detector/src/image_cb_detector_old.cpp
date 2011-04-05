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

#include <ros/ros.h>
#include <image_cb_detector/image_cb_detector_old.h>
#include <visual_pose_estimation/pose_estimator.h>

using namespace image_cb_detector;
using namespace std;

bool ImageCbDetectorOld::configure(const ConfigGoal& config)
{
  config_ = config;

  return true;
}



bool ImageCbDetectorOld::detect(const sensor_msgs::ImageConstPtr& ros_image,
                                const sensor_msgs::CameraInfoConstPtr& info_msg,
                                calibration_msgs::CalibrationPattern& result)
{
  IplImage *image = NULL;
  try
  {
    image = bridge_.imgMsgToCv(ros_image, "mono8");
  }
  catch (sensor_msgs::CvBridgeException error)
  {
    ROS_ERROR("error");
    return false;
  }

  // \todo This code has been pretty much copied from laser_cb_detector. (Wow... this is poor software design)

  // ***** Resize the image based on scaling parameters in config *****
  const int scaled_width  = (int) (.5 + image->width  * config_.width_scaling);
  const int scaled_height = (int) (.5 + image->height * config_.height_scaling);
  IplImage* image_scaled = cvCreateImage(cvSize( scaled_width, scaled_height),
                                         image->depth,
                                         image->nChannels);
  cvResize(image, image_scaled, CV_INTER_LINEAR);

  // ***** Allocate vector for found corners *****
  vector<CvPoint2D32f> cv_corners;
  cv_corners.resize(config_.num_x*config_.num_y);

  // ***** Do the actual checkerboard extraction *****
  CvSize board_size = cvSize(config_.num_x, config_.num_y);
  int num_corners ;
  int found = cvFindChessboardCorners( image_scaled, board_size, &cv_corners[0], &num_corners,
                                       CV_CALIB_CB_ADAPTIVE_THRESH) ;

  if(found)
  {
    ROS_DEBUG("Found CB");
    //ROS_INFO("Found checkerboard. Subpixel window: %i", config_.subpixel_window);


    CvSize subpixel_window    = cvSize(config_.subpixel_window,
                                       config_.subpixel_window);
    CvSize subpixel_zero_zone = cvSize(config_.subpixel_zero_zone,
                                       config_.subpixel_zero_zone);

    // Subpixel fine-tuning stuff
    cvFindCornerSubPix(image_scaled, &cv_corners[0], num_corners,
                       subpixel_window,
                       subpixel_zero_zone,
                       cvTermCriteria(CV_TERMCRIT_ITER,20,1e-2));
  }
  else
    ROS_DEBUG("Didn't find CB");
  cvReleaseImage(&image_scaled);

  // ***** Downscale the detected corners and generate the CalibrationPattern message *****
  result.header.stamp    = ros_image->header.stamp;
  result.header.frame_id = ros_image->header.frame_id;

  result.object_points.resize(config_.num_x * config_.num_y);
  for (unsigned int i=0; i < config_.num_y; i++)
  {
    for (unsigned int j=0; j < config_.num_x; j++)
    {
      result.object_points[i*config_.num_x + j].x = j*config_.spacing_x;
      result.object_points[i*config_.num_x + j].y = i*config_.spacing_y;
      result.object_points[i*config_.num_x + j].z = 0.0;
    }
  }

  result.image_points.resize(num_corners);

  for (int i=0; i < num_corners; i++)
  {
    result.image_points[i].x = cv_corners[i].x / config_.width_scaling;
    result.image_points[i].y = cv_corners[i].y / config_.height_scaling;
  }

  if (found)
  {
    // Do the pose fitting
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(*info_msg);

    // Build checkerboard model for estimator
    cv::Mat_<cv::Vec3f> grid_points(result.object_points.size(), 1);
    for (unsigned int i=0; i < result.object_points.size(); i++)
      grid_points(i, 0) = cv::Vec3f(result.object_points[i].x, result.object_points[i].y, result.object_points[i].z);
    visual_pose_estimation::PoseEstimator estimator(grid_points);

    // Build measurement vector
    std::vector<cv::Point2f> points(result.image_points.size());
    for (unsigned int i=0; i < result.image_points.size(); i++)
      points[i] = cv::Point2f(result.image_points[i].x, result.image_points[i].y);

    // Do the actual pose solving
    tf::Pose tf_pose = estimator.solve(points, cam_model);
    geometry_msgs::PoseStamped pose_msg;
    tf::poseTFToMsg(tf_pose, pose_msg.pose);
    pose_msg.header.stamp    = ros_image->header.stamp;
    pose_msg.header.frame_id = ros_image->header.frame_id;
    result.object_pose = pose_msg;
  }

  result.success = found;

  return true;
}
