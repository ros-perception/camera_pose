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

#ifndef DENSE_LASER_ASSEMBLER_DENSE_LASER_ASSEMBLER_H_
#define DENSE_LASER_ASSEMBLER_DENSE_LASER_ASSEMBLER_H_

#include <settlerlib/sorted_deque.h>

// Messages
#include <sensor_msgs/LaserScan.h>
#include <calibration_msgs/DenseLaserSnapshot.h>

namespace dense_laser_assembler
{

class DenseLaserAssembler
{
public:
  DenseLaserAssembler(const unsigned int cache_size = 100);

  void setCacheSize(const unsigned int cache_size);

  void add(const sensor_msgs::LaserScanConstPtr& laser_scan);

  /**
   * \brief Takes a vector of LaserScan messages, and composes them into one larger snapshot.
   * \param start The earliest scan time to be included in the snapshot
   * \param end The latest scan time to be included in the snapshot
   * \param output: A populated snapshot message
   * \return True on success
   */
  bool assembleSnapshot(const ros::Time& start, const ros::Time& end, calibration_msgs::DenseLaserSnapshot& snapshot);

private:

  //! Stores the history of laser scans
  settlerlib::SortedDeque<sensor_msgs::LaserScanConstPtr> cache_;


  /**
   * \brief Takes a vector of LaserScan messages, and composes them into one larger snapshot.
   * \param scans Vector of laser scans
   * \param snapshot Output: A populated snapshot message
   * \return True on success. Could fail if all the scans don't have the same metadata
   */
  bool flattenScanVec(const std::vector< sensor_msgs::LaserScanConstPtr >& scans,
                      calibration_msgs::DenseLaserSnapshot& snapshot);

  /**
   * \brief Internal data consistency check
   *
   * A check to make sure that a scan's metadata matches the metadata of the snapshot.
   * \param snapshot The snapshot to compare against
   * \param scan the scan to compare against
   * \return True on success. False if metadata doesn't match
   */
  bool verifyMetadata(const calibration_msgs::DenseLaserSnapshot& snapshot, const sensor_msgs::LaserScan& scan);

};

}

#endif
