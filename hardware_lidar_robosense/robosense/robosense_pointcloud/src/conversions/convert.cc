
/** @file

    This class converts raw Robosense 3D LIDAR packets to PointCloud2.

*/

#include "robosense_pointcloud/convert.h"

#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>
namespace robosense_pointcloud {
/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh)
    : data_(new robosense_rawdata::RawData()),
      outMsg(new robosense_rawdata::VPointCloud()) {
  private_nh.param("frame_id_base", frame_id_base_, std::string("car"));
  private_nh.param("fixed_frame_id", fixed_frame_id_, std::string("enu"));
  private_nh.param("frame_id", frame_id_, std::string("robosense_center"));

  std::string packet_topic, points_topic_array;
  private_nh.param("packet_topic", packet_topic,
                   std::string("/sensor/lidar/packet"));
  private_nh.param("points_topic_convert", points_topic_array,
                   std::string("/sensor/lidar/point_cloud_no_compensate"));
  private_nh.param("pointcloud_width", width_, 0);
  private_nh.param("pointcloud_height", height_, 0);
  double offset = 0;
  private_nh.param("motion_compensate_offset", offset, 0.);
  motion_compensate_offset_ = ros::Duration(offset);
  data_->setup(private_nh);

  // advertise output point cloud (before subscribing to input data)
  output_ = node.advertise<sensor_interface_msgs::Pointcloud2>(points_topic_array, 10);

  srv_ = boost::make_shared<
      dynamic_reconfigure::Server<robosense_pointcloud::CloudNodeConfig>>(
      private_nh);
  dynamic_reconfigure::Server<
      robosense_pointcloud::CloudNodeConfig>::CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  // subscribe to RobosensePacket packets
  robosense_packet_ =
      node.subscribe(packet_topic, 100, &Convert::processScan, this);
  // Diagnostics
  diagnostics_.setHardwareID("Robosense Convert");
  // Arbitrary frequencies since we don't know which RPM is used, and are only
  // concerned about monitoring the frequency.
  diag_min_freq_ = 2.0;
  diag_max_freq_ = 20.0;
  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic(
      "robosense_points", diagnostics_,
      FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
      TimeStampStatusParam()));
}

void Convert::callback(robosense_pointcloud::CloudNodeConfig &config,
                       uint32_t level) {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.view_direction, config.view_width);
}

void Convert::initmsg() {
  outMsg->points.clear();
  outMsg->points.resize(width_ * height_);
  outMsg->header.frame_id = frame_id_base_;
  outMsg->height = height_;
  outMsg->width = width_;

  if (initialization_successful_)
    return;

  tf::StampedTransform tf_car2lidar;
  try {
    listener_.lookupTransform(frame_id_base_, // default [car]
                              frame_id_,      // default [robosense_center]
                              ros::Time(0), tf_car2lidar);
    initialization_successful_ = true;
    is3d_car2lidar_ = TF2Eigen(tf_car2lidar);
  } catch (tf::TransformException &ex) {
    ROS_ERROR_THROTTLE(1, "Can not lookup transform from lidar to car: %s",
                       ex.what());
    // ROS_ERROR("Can not lookup transform from lidar to car . %s", ex.what());
    return;
  }
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(
    const sensor_interface_msgs::RobosensePacket::ConstPtr &packetMsg) {
  // if (output_.getNumSubscribers() == 0) // no one listening?
  // return;                             // avoid much work
  if (!initialization_successful_) {
    initmsg();
    return;
  }
  static int history_id = 0;

  if (history_id > packetMsg->id) {
    ros::Time tmp_time;
    pcl_conversions::toPCL(tmp_time.fromNSec(packetMsg->stamp) + motion_compensate_offset_,
                           outMsg->header.stamp);
    output_.publish(outMsg);
    data_->reset_startingangle();
    initmsg();
  }
  data_->unpack_rsruby128(*packetMsg, *outMsg, is3d_car2lidar_);
  history_id = packetMsg->id;
}

} // namespace robosense_pointcloud
