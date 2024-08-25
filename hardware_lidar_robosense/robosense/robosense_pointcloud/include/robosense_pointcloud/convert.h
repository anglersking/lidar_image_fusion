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

    This class converts raw Robosense 3D LIDAR packets to PointCloud2.

*/

#ifndef ROBOSENSE_POINTCLOUD_CONVERT_H
#define ROBOSENSE_POINTCLOUD_CONVERT_H

#include "eigen_utils.h"
#include "message_filters/subscriber.h"
#include "std_msgs/Bool.h"
#include "tf/message_filter.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <pcl_ros/impl/transforms.hpp>
#include <ros/ros.h>
#include <sensor_interface_msgs/Pointcloud2.h>
#include <string>
#include <sys/time.h>
#include <tf_conversions/tf_eigen.h>
#include <vector>
#include <robosense_pointcloud/CloudNodeConfig.h>
#include <robosense_pointcloud/TransformNodeConfig.h>
#include <robosense_pointcloud/pointcloudXYZIR.h>
#include <robosense_pointcloud/rawdata.h>

namespace robosense_pointcloud {
class Convert {
public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~Convert() {}

private:
  void callback(robosense_pointcloud::CloudNodeConfig &config, uint32_t level);
  void processScan(const sensor_interface_msgs::RobosensePacket::ConstPtr &scanMsg);

  // Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<
      dynamic_reconfigure::Server<robosense_pointcloud::CloudNodeConfig>>
      srv_;

  boost::shared_ptr<robosense_rawdata::RawData> data_;
  ros::Subscriber robosense_packet_;
  ros::Publisher output_;

  /// configuration parameters
  typedef struct {
    int npackets; // number of packets to combine
  } Config;
  Config config_;

  // diagnostics updater
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;

  // add
  void initmsg();
  boost::shared_ptr<robosense_rawdata::VPointCloud> outMsg;
  int width_, height_;
  tf::TransformListener listener_;
  bool initialization_successful_ = false;
  std::string frame_id_;
  std::string frame_id_base_;
  std::string fixed_frame_id_;
  ros::Duration motion_compensate_offset_;
  Eigen::Isometry3d is3d_car2lidar_;
};
} // namespace robosense_pointcloud

#endif // ROBOSENSE_POINTCLOUD_CONVERT_H
