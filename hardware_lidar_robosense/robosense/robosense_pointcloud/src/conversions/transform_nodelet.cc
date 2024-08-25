/** @file

    This ROS nodelet transforms raw Robosense 3D LIDAR packets to a
    PointCloud2 in the /enu frame.

*/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "robosense_pointcloud/transform.h"

namespace robosense_pointcloud {
class TransformNodelet : public nodelet::Nodelet {
public:
  TransformNodelet() {}
  ~TransformNodelet() {}

private:
  virtual void onInit();
  boost::shared_ptr<Transform> tf_;
};

/** @brief Nodelet initialization. */
void TransformNodelet::onInit() {
  tf_.reset(new Transform(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace robosense_pointcloud

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(robosense_pointcloud::TransformNodelet, nodelet::Nodelet)
