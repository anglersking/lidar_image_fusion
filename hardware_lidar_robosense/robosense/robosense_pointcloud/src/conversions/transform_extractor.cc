#include "robosense_pointcloud/transform_extractor.h"
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <string>
#include <tf_conversions/tf_eigen.h>
namespace robosense_pointcloud
{
  // only for vlp128
  const int width_ = 1875;
  const int height_ = 128;
  const int rpm_ = 600;
  const float start_angle_ = 180;
  const double offset_ = -0.05;

  Transform_Extractor::Transform_Extractor()
      : data_(new robosense_rawdata::RawData()),
        outMsg(new robosense_rawdata::VPointCloud())
  {
    data_->setupOffline(width_, height_, rpm_, start_angle_);
  }

  robosense_rawdata::VPointCloud::Ptr Transform_Extractor::extractorScan(
      const sensor_interface_msgs::LidarScan::ConstPtr &scanMsg,
      const geometry_msgs::TransformStamped tf)
  {
    outMsg->points.clear();
    outMsg->points.resize(width_ * height_);
    outMsg->height = height_;
    outMsg->width = width_;
    ros::Duration motion_compensate_offset_ = ros::Duration(offset_);
    Eigen::Isometry3d is3d_car2lidar = Eigen::Isometry3d::Identity();
    is3d_car2lidar = Statictf2Eigen(tf);
    for (size_t next = 0; next < scanMsg->robosense_packets.size(); ++next)
    {
      data_->unpack_rsruby128(scanMsg->robosense_packets[next], *outMsg, is3d_car2lidar);
    }
    if (outMsg->points.size())
    {
      pcl_conversions::toPCL(scanMsg->header.stamp + motion_compensate_offset_,
                             outMsg->header.stamp);
      data_->reset_startingangle();
      return outMsg;
    }
    else
    {
      data_->reset_startingangle();
      return NULL;
    }
  }
} // namespace robosense_pointcloud
