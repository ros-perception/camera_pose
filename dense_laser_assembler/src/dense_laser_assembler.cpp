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

#include "dense_laser_assembler/dense_laser_assembler.h"

using namespace std;
using namespace dense_laser_assembler;

DenseLaserAssembler::DenseLaserAssembler(const unsigned int cache_size)
  : cache_(&settlerlib::SortedDeque<sensor_msgs::LaserScanConstPtr>::getPtrStamp, "dense_laser_deque")
{
  setCacheSize(cache_size);
}

void DenseLaserAssembler::setCacheSize(const unsigned int cache_size)
{
  cache_.setMaxSize(cache_size);
}

void DenseLaserAssembler::add(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  cache_.add(laser_scan);
}

bool DenseLaserAssembler::assembleSnapshot(const ros::Time& start, const ros::Time& end, calibration_msgs::DenseLaserSnapshot& snapshot)
{
  vector< sensor_msgs::LaserScanConstPtr > scan_vec;
  ROS_DEBUG("Assembling snapshot:\n"
            "   from: %.2f\n"
            "   to:   %.2f\n"
            "   duration: %.3f\n",
            start.toSec(), end.toSec(), (end-start).toSec());

  scan_vec = cache_.getInterval(start, end);
  return flattenScanVec(scan_vec, snapshot);
}

bool DenseLaserAssembler::flattenScanVec(const std::vector< sensor_msgs::LaserScanConstPtr >& scans,
                                         calibration_msgs::DenseLaserSnapshot& snapshot)
{
  if (scans.size() == 0)
  {
    ROS_WARN("Trying to build empty cloud") ;
    snapshot.angle_min       = 0.0;
    snapshot.angle_max       = 0.0;
    snapshot.angle_increment = 0.0;
    snapshot.time_increment  = 0.0;
    snapshot.range_min       = 0.0;
    snapshot.range_max       = 0.0;
    snapshot.readings_per_scan = 0;
    snapshot.num_scans       = 0;
    snapshot.ranges.clear();
    snapshot.intensities.clear();
    snapshot.scan_start.clear();
    return true;
  }

  // Not really sure what the overall stamp should be.  But for now, copy over the stamp from the last scan
  snapshot.header.stamp = scans[scans.size()-1]->header.stamp;

  // Fill in all the metadata
  snapshot.header.frame_id = scans[0]->header.frame_id;
  snapshot.angle_min       = scans[0]->angle_min;
  snapshot.angle_max       = scans[0]->angle_max;
  snapshot.angle_increment = scans[0]->angle_increment;
  snapshot.time_increment  = scans[0]->time_increment;
  snapshot.range_min       = scans[0]->range_min;
  snapshot.range_max       = scans[0]->range_max;

  // Define the data dimensions
  snapshot.readings_per_scan = scans[0]->ranges.size();
  snapshot.num_scans         = scans.size();
  const unsigned int& w = snapshot.readings_per_scan;
  const unsigned int& h = snapshot.num_scans;

  // Do a consistency check on the metadata
  for (unsigned int i=0; i<scans.size(); i++)
  {
    if (!verifyMetadata(snapshot, *scans[i]))
    {
      ROS_WARN("Metadata doesn't match. It is likely that someone just changed the laser's configuration");
      return false ;
    }
  }

  // Allocate data vectors
  snapshot.scan_start.resize(h) ;
  snapshot.ranges.resize(w*h) ;
  snapshot.intensities.resize(w*h) ;

  const unsigned int range_elem_size     = sizeof(scans[0]->ranges[0]) ;
  const unsigned int intensity_elem_size = sizeof(scans[0]->intensities[0]) ;

  // Make sure our sizes match before doing memcpy
  assert(sizeof(snapshot.ranges[0])      == sizeof(scans[0]->ranges[0]));
  assert(sizeof(snapshot.intensities[0]) == sizeof(scans[0]->intensities[0]));

  for (unsigned int i=0; i<h; i++)
  {
    memcpy(&snapshot.ranges[i*w],      &scans[i]->ranges[0],      w*range_elem_size) ;
    memcpy(&snapshot.intensities[i*w], &scans[i]->intensities[0], w*intensity_elem_size) ;

    // Copy time stamp
    snapshot.scan_start[i] = scans[i]->header.stamp ;
  }

  ROS_DEBUG("Done building snapshot that is [%u rows] x [%u cols]", h, w) ;
  ROS_DEBUG("  ranges.size      = %u", snapshot.ranges.size());
  ROS_DEBUG("  intensities.size = %u", snapshot.intensities.size());

  return true ;
}

static const double eps = 1e-9 ;

#define CHECK(a) \
{ \
  if ( (snapshot.a - scan.a < -eps) || (snapshot.a - scan.a > eps)) \
    return false ; \
}

bool DenseLaserAssembler::verifyMetadata(const calibration_msgs::DenseLaserSnapshot& snapshot, const sensor_msgs::LaserScan& scan)
{
  CHECK(angle_min) ;
  CHECK(angle_max) ;
  CHECK(angle_increment) ;
  CHECK(time_increment) ;
  CHECK(range_min) ;
  CHECK(range_max) ;

  if (snapshot.header.frame_id.compare(scan.header.frame_id) != 0)
    return false ;

  if (snapshot.readings_per_scan != scan.ranges.size())
    return false ;
  if (snapshot.readings_per_scan != scan.intensities.size())
    return false ;


  return true ;
}
