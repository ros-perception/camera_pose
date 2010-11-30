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

#include <laser_joint_processor/joint_image_interpolator.h>

using namespace laser_joint_processor;
using namespace std;

bool JointImageInterpolator::interp(const std::vector <calibration_msgs::ImagePoint>& points,
                                    IplImage* image, std::vector<float>& positions, std::vector<float>& velocities)
{
  const unsigned int N = points.size();

  // Do consistency checks
  if (image->depth != IPL_DEPTH_32F)
  {
    ROS_ERROR("Expecting input image to have depth of IPL_DEPTH_32F");
    return false;
  }
  if (image->nChannels != 2)
  {
    ROS_ERROR("Expecting input image to have 2 channels. Instead had %i channels", image->nChannels);
    return false;
  }

  // Allocate Maps
  vector<float> map_x_vec(N);
  vector<float> map_y_vec(N);
  CvMat map_x_mat = cvMat(N, 1, CV_32FC1, &map_x_vec[0]);
  CvMat map_y_mat = cvMat(N, 1, CV_32FC1, &map_y_vec[0]);

  // Set up maps
  for (unsigned int i=0; i<N; i++)
  {
    map_x_vec[i] = points[i].x;
    map_y_vec[i] = points[i].y;
  }

  // Allocate Destination Image
  vector<float> dest_vec(2*N);
  CvMat dest_mat = cvMat(N, 1, CV_32FC2, &dest_vec[0]);

  // Perform the OpenCV interpolation
  cvRemap(image, &dest_mat, &map_x_mat, &map_y_mat);

  // Copy results into output vectors
  positions.resize(N);
  velocities.resize(N);
  for (unsigned int i=0; i<N; i++)
  {
    positions[i]  = dest_vec[2*i+0];
    velocities[i] = dest_vec[2*i+1];
  }

  return true;
}


bool laser_joint_processor::interpSnapshot(const std::vector <calibration_msgs::ImagePoint>& points,
                                           const calibration_msgs::DenseLaserSnapshot& snapshot,
                                           std::vector<float>& angles,
                                           std::vector<float>& ranges)
{
  const unsigned int N = points.size();

  // Check to make sure points are in range
  for (unsigned int i=0; i<N; i++)
  {
    if (points[i].x < 0  ||
        points[i].x > snapshot.readings_per_scan-1 ||
        points[i].y < 0 ||
        points[i].y > snapshot.num_scans-1 )
    {
      return false;
    }
  }

  // Set up input image
  CvMat range_image = cvMat(snapshot.num_scans, snapshot.readings_per_scan, CV_32FC1, (void*) &snapshot.ranges[0]);

  // Allocate Maps
  vector<float> map_x_vec(N);
  vector<float> map_y_vec(N);
  CvMat map_x_mat = cvMat(N, 1, CV_32FC1, &map_x_vec[0]);
  CvMat map_y_mat = cvMat(N, 1, CV_32FC1, &map_y_vec[0]);

  // Set up maps
  for (unsigned int i=0; i<N; i++)
  {
    map_x_vec[i] = points[i].x;
    map_y_vec[i] = points[i].y;
  }

  // Allocate Destination Image
  ranges.resize(N);
  CvMat ranges_mat = cvMat(N, 1, CV_32FC1, &ranges[0]);

  // Perform the OpenCV interpolation
  cvRemap(&range_image, &ranges_mat, &map_x_mat, &map_y_mat);

  // Do angle interp manually
  angles.resize(N);
  for (unsigned int i=0; i<N; i++)
    angles[i] = snapshot.angle_min +  points[i].x*snapshot.angle_increment;

  return true;
}





