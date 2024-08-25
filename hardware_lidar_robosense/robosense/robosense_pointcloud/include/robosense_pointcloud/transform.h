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

    This class transforms raw Robosense 3D LIDAR packets to PointCloud2
    in the /enu frame of reference.

*/

#ifndef ROBOSENSE_POINTCLOUD_TRANSFORM_H
#define ROBOSENSE_POINTCLOUD_TRANSFORM_H

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
#include <sensor_interface_msgs/LidarScan.h>
#include <robosense_pointcloud/TransformNodeConfig.h>
#include <robosense_pointcloud/pointcloudXYZIR.h>
//#include <robosense_pointcloud/robosense_pointcloud_log.h>
#include <glog/logging.h>
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

namespace robosense_pointcloud {

typedef robosense_pointcloud::PointXYZIT VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

using TransformNodeCfg = robosense_pointcloud::TransformNodeConfig;

class Transform {
public:
  Transform(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~Transform() { google::ShutdownGoogleLogging(); }

  inline long long get_timenow() {
    long long timenow;
    struct timeval ce;
    gettimeofday(&ce, NULL);
    timenow = ce.tv_sec * 1000000 + ce.tv_usec;
    return timenow;
  }

private:
  std::string frame_id_base_;
  std::string fixed_frame_id_;
  ros::Duration motion_compensate_offset_;
  // int lidar_rings_;

  void processScan(const sensor_interface_msgs::LidarScan::ConstPtr &scanMsg);

  bool update_stamptf(const tf::StampedTransform &stampedtransform);
  bool infer_tf(const ros::Time stamp, Eigen::Isometry3d &trans);
  // Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<
      dynamic_reconfigure::Server<robosense_pointcloud::TransformNodeConfig>>
      srv_;
  void reconfigure_callback(robosense_pointcloud::TransformNodeConfig &config,
                            uint32_t level);

  const std::string tf_prefix_;
  boost::shared_ptr<robosense_rawdata::RawData> data_;
  boost::shared_ptr<robosense_rawdata::VPointCloud> outMsg;
  message_filters::Subscriber<sensor_interface_msgs::LidarScan> lidar_scan_;
  tf::MessageFilter<sensor_interface_msgs::LidarScan> *tf_filter_;
  ros::Publisher output_;
  tf::TransformListener listener_;

  ros::Subscriber sub_for_stamp_;
  ros::Time prev_stamp_;
  void stamp_checker(const sensor_interface_msgs::LidarScan::ConstPtr &msg);
  /// configuration parameters
  typedef struct {
    std::string frame_id; // fixed frame ID
  } Config;
  Config config_;

  /// tf_queue
  std::mutex que_mutex_;
  double stamp_tolerance = 0.11;
  double delay_tolerance = 0.50;
  typedef struct {
    Eigen::Isometry3d is3d;
    ros::Time stamp;
    Eigen::Vector3d vec;
  } Stamp_tf;

  std::deque<Stamp_tf> tf_que_;
  unsigned int max_queuesize_ = 64;

  // Point cloud buffers for collecting points within a packet.  The
  // packetcloud_ are class members only to avoid reallocation on
  // every message.
  // const int peckets_size_ = 625;
  ros::Publisher pubCompensation;

  int width_, height_;
  // std::vector<PointcloudXYZIR> Clouds;
  // diagnostics updater
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
};
} // namespace robosense_pointcloud

#endif // ROBOSENSE_POINTCLOUD_TRANSFORM_H
