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

#include <cstdio>
#include <laser_joint_processor/joint_imager.h>

using namespace laser_joint_processor;
using namespace std;

JointImager::JointImager()
{



}

JointImager::~JointImager()
{
  // Deallocate everything
  for (unsigned int i=0; i<images.size(); i++)
    cvReleaseImage(&images[i]);
}

bool JointImager::update(const calibration_msgs::DenseLaserSnapshot& snapshot,
                         const settlerlib::SortedDeque<joint_states_settler::DeflatedJointStates>& cache,
                         const ros::Duration& max_interp)
{
  if (cache.size() == 0)
    return false;

  unsigned int num_channels = cache.back().channels_.size();

  // Size the openCV datatypes accordingly
  allocateImages(snapshot.num_scans, snapshot.readings_per_scan, num_channels);

  vector<double> cur_positions;

  //bool success;

  unsigned int after_index = 0;
  for (unsigned int i=0; i<snapshot.num_scans; i++)
  {
    for (unsigned int j=0; j<snapshot.readings_per_scan; j++)
    {
      ros::Time pixel_time = snapshot.scan_start[i] + ros::Duration(snapshot.time_increment * j);
      while(after_index < cache.size() && cache[after_index].header.stamp < pixel_time)
        after_index++;
      assert(after_index < cache.size());    // Should never encounter this, if we already did our bounds checking on the snapshot
      assert(after_index > 0);

      // Sanity check: Determine how far apart are the elems we're interpolating between
      ros::Duration interp_diff = cache[after_index].header.stamp - cache[after_index-1].header.stamp;
      if (interp_diff > max_interp)
      {
        ROS_WARN("Interpolating between elems that are [%.2fs] apart. Probably dopped some messages. Going to ignore this snapshot", interp_diff.toSec());
        return false;
      }

      settlerlib::Deflated::interp(cache[after_index-1], cache[after_index], pixel_time, cur_positions);

      // Populate position field [Channel 0] in each joint image
      for (unsigned int k=0;k<num_channels;k++)
        *(((float*)(images[k]->imageData + images[k]->widthStep*i))+2*j + 0) = cur_positions[k];
    }
    ros::Time scan_start_time = snapshot.scan_start[i];
    ros::Time scan_end_time   = snapshot.scan_start[i] + ros::Duration(snapshot.time_increment * (snapshot.readings_per_scan-1));

    for (unsigned int k=0; k<num_channels; k++)
    {
      float start_pos = *(((float*)(images[k]->imageData + images[k]->widthStep*i))+2*0 + 0);
      float end_pos   = *(((float*)(images[k]->imageData + images[k]->widthStep*i))+2*(snapshot.readings_per_scan-1) + 0);
      float vel = (end_pos - start_pos) / (scan_end_time-scan_start_time).toSec();
      // Walk along image row, populating the velocity channel. (Every other float is a velocity).
      float* vel_ptr = (float*)(images[k]->imageData + images[k]->widthStep*i) + 2*0 + 1;
      for (unsigned int j=0; j<snapshot.readings_per_scan; j++)
      {
        *vel_ptr = vel;
        vel_ptr += 2;
      }
    }
  }

  /* (Inefficient implementation)
  // Iterate over scan
  for (unsigned int i=0; i<snapshot.num_scans; i++)
  {
    ros::Time scan_start_time = snapshot.scan_start[i];
    ros::Time scan_end_time   = snapshot.scan_start[i] + ros::Duration(snapshot.time_increment * (snapshot.readings_per_scan-1));

    success = computeVelocity(scan_start_time, scan_end_time, cache, cur_velocities);
    if (!success)
    {
      ROS_ERROR("Error computing velocity");
      return false;
    }

    // Iterate over each ray in a scan
    for (unsigned int j=0; j<snapshot.readings_per_scan; j++)
    {
      ros::Time pixel_time = snapshot.scan_start[i] + ros::Duration(snapshot.time_increment * j);
      success = interpPosition(pixel_time, cache, cur_positions);
      if (!success)
      {
        ROS_ERROR("Error interpolating for pixel position");
        return false;
      }
      for (unsigned int k=0;k<num_channels;k++)
      {
        *(((float*)(images[k]->imageData + images[k]->widthStep*i))+2*j + 0) = cur_positions[k];
        *(((float*)(images[k]->imageData + images[k]->widthStep*i))+2*j + 1) = cur_velocities[k];
      }
    }
  }
  */
  return true;
}

void JointImager::displayImage(unsigned int i)
{
  IplImage* image = getJointImage(i);

  for (int i=0; i<image->height; i++)
  {
    for (int j=0; j<image->width; j++)
    {
      printf("%5.2f  ", *((float*)(image->imageData + i*image->widthStep) + j*image->nChannels));
    }
    printf("\n");
  }
}

void JointImager::writeImage(unsigned int i, const string& filename)
{
  FILE* file = fopen(filename.c_str(), "w");

  if (file)
    printf("About to write to file %s\n", filename.c_str());

  IplImage* image = getJointImage(i);

  for (int i=0; i<image->height; i++)
  {
    for (int j=0; j<image->width; j++)
    {
      fprintf(file, "% 3.2f  ", *((float*)(image->imageData + i*image->widthStep) + j*image->nChannels));
    }
    fprintf(file, "\n");
  }

  fclose(file);
}


bool JointImager::interpPosition(const ros::Time& target,
                                 const settlerlib::SortedDeque<joint_states_settler::DeflatedJointStates>& cache,
                                 vector<double>& result)
{
  bool success;
  joint_states_settler::DeflatedJointStates before;
  joint_states_settler::DeflatedJointStates after;

  success = cache.getElemBeforeTime(target, before);
  if (!success)
  {
    ROS_ERROR("Couldn't find elem before time");
    return false;
  }
  success = cache.getElemAfterTime(target, after);
  if (!success)
  {
    ROS_ERROR("Couldn't find elem after time");
    return false;
  }
  if (before.channels_.size() != after.channels_.size())
  {
    ROS_ERROR("# of joints has changed in the middle of a run");
    return false;
  }

  success = settlerlib::Deflated::interp(before, after, target, result);
  if (!success)
  {
    ROS_ERROR("Error performing interpolation");
    return false;
  }

  return true;
}

bool JointImager::computeVelocity(const ros::Time& start, const ros::Time& end,
                                  const settlerlib::SortedDeque<joint_states_settler::DeflatedJointStates>& cache,
                                  vector<double>& result)
{
  vector<double> start_positions;
  vector<double> end_positions;

  bool success;
  success = interpPosition(start, cache, start_positions);
  if (!success)
  {
    ROS_ERROR("Error extracting start position");
    return false;
  }

  success = interpPosition(end,   cache, end_positions);
  if (!success)
  {
    ROS_ERROR("Error extracting start position");
    return false;
  }

  if (start_positions.size() != end_positions.size())
  {
    ROS_ERROR("# of joints has changed during run.  Can't compute velocity between samples");
    return false;
  }

  const unsigned int N = start_positions.size();

  result.resize(N);
  for (unsigned int i=0; i<N; i++)
    result[i] = (end_positions[i] - start_positions[i]) / (end-start).toSec();

  return true;
}

void JointImager::allocateImages(unsigned int height, unsigned int width, unsigned int channels)
{
  // Deallocate everything
  for (unsigned int i=0; i<images.size(); i++)
    cvReleaseImage(&images[i]);

  images.resize(channels);

  CvSize image_size = cvSize(width, height);

  for (unsigned int i=0; i<channels; i++)
    images[i] = cvCreateImage(image_size, IPL_DEPTH_32F, 2);
}

IplImage* JointImager::getJointImage(unsigned int index) const
{
  return images[index];
}
