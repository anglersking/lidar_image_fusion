/*
 *  Copyright (C) 2012 Austin Robot Technology,
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This ROS nodelet converts raw Robosense 3D LIDAR packets to a
    PointCloud2.

*/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "robosense_pointcloud/convert.h"

namespace robosense_pointcloud {
class CloudNodelet : public nodelet::Nodelet {
public:
  CloudNodelet() {}
  ~CloudNodelet() {}

private:
  virtual void onInit();
  boost::shared_ptr<Convert> conv_;
};

/** @brief Nodelet initialization. */
void CloudNodelet::onInit() {
  conv_.reset(new Convert(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace robosense_pointcloud

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(robosense_pointcloud::CloudNodelet, nodelet::Nodelet)
