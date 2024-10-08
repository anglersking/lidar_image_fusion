// Copyright (C) 2009, 2010, 2011, 2012, 2019 Austin Robot Technology, Jack
// O'Quin, Jesse Vera, Joshua Whitley All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** @file

    This class transforms raw Velodyne 3D LIDAR packets to PointCloud2
    in the /enu frame of reference.

*/

#ifndef ROBOSENSE_POINTCLOUD_EXTRACTOR_H
#define ROBOSENSE_POINTCLOUD_EXTRACTOR_H

#include "eigen_utils.h"
#include "message_filters/subscriber.h"
#include "std_msgs/Bool.h"
#include "tf/message_filter.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_interface_msgs/Pointcloud2.h>
#include <robosense_pointcloud/pointcloudXYZIR.h>
#include <robosense_pointcloud/rawdata.h>
// include template implementations to transform a custom point cloud
#include <mutex>
#include <pcl_ros/impl/transforms.hpp>
#include <string>
#include <sys/time.h>
#include <vector>

// instantiate template for transforming a VPointCloud
template bool pcl_ros::transformPointCloud<robosense_rawdata::VPoint>(
    const std::string &, const robosense_rawdata::VPointCloud &,
    robosense_rawdata::VPointCloud &, const tf::TransformListener &);

namespace robosense_pointcloud
{
  typedef robosense_pointcloud::PointXYZIT VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  class Transform_Extractor
  {
  public:
    Transform_Extractor(); // for bag extractor
    ~Transform_Extractor() {}

    inline long long get_timenow()
    {
      long long timenow;
      struct timeval ce;
      gettimeofday(&ce, NULL);
      timenow = ce.tv_sec * 1000000 + ce.tv_usec;
      return timenow;
    }
    robosense_rawdata::VPointCloud::Ptr
    extractorScan(const sensor_interface_msgs::LidarScan::ConstPtr &scanMsg,
                  const geometry_msgs::TransformStamped tf); // for bag extractor
    robosense_rawdata::VPointCloud::Ptr outMsg = nullptr;

  private:
    boost::shared_ptr<robosense_rawdata::RawData> data_;
  };
} // namespace robosense_pointcloud

#endif // ROBOSENSE_POINTCLOUD_EXTRACTOR_H
