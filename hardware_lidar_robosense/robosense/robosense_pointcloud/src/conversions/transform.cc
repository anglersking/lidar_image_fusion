/** @file

    This class transforms raw Robosense 3D LIDAR packets to PointCloud2
    in the /enu frame of reference.

    @author Jack O'Quin
    @author Jesse Vera

*/

#include "robosense_pointcloud/transform.h"
#include "mlog/mlog.h"
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <string>
#include <tf_conversions/tf_eigen.h>
namespace robosense_pointcloud {
/** @brief Constructor. */
Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh)
    : tf_prefix_(tf::getPrefixParam(private_nh)), // launch参数读取
      data_(new robosense_rawdata::RawData()),
      outMsg(new robosense_rawdata::VPointCloud()) {

  google::InitGoogleLogging("");
  std::string glog_dir =
      ros::package::getPath("robosense_pointcloud") + "/logs/";
  google::SetLogDestination(google::GLOG_INFO, glog_dir.c_str());
  // FLAGS_log_dir = glog_dir;
  google::SetStderrLogging(google::GLOG_WARNING);
  google::SetLogFilenameExtension("transform_node_log_");

  // FLAGS_colorlogtostderr = true;          // Set log color
  FLAGS_logbufsecs = 0;                   // Set log output speed(s)
  FLAGS_max_log_size = 1024;              // Set max log file size
  FLAGS_stop_logging_if_full_disk = true; // If disk is full
  FLAGS_alsologtostderr = 0;
  // FLAGS_minloglevel = google::ERROR;

  //init mlog
  auto settings = mlog::MLogManager::get_settings();
	settings.endpoint = mlog::MLOG_ENDPOINT_ROS_TOPIC;
	settings.export_frequence = 0;
  settings.level = mlog::MLogLevel::MLOG_LEVEL_INFO;
	settings.rostopic_name = "/robosense/transform_logs";
	mlog::MLogManager::set_settings(settings);

  private_nh.param("frame_id_base", frame_id_base_, std::string("car"));
  private_nh.param("fixed_frame_id", fixed_frame_id_, std::string("enu"));
  std::string packet_topic, points_topic_array;
  private_nh.param("packet_topic", packet_topic,
                   std::string("/sensor/lidar/velodyne_multi_scans"));
  private_nh.param("points_topic_transform", points_topic_array,
                   std::string("robosense_points"));
  private_nh.param("pointcloud_width", width_, 0);
  private_nh.param("pointcloud_height", height_, 0);
  double offset = 0;
  private_nh.param("motion_compensate_offset", offset, 0.);
  motion_compensate_offset_ = ros::Duration(offset);

  // Read calibration.
  data_->setup(private_nh);

  // advertise output point cloud (before subscribing to input data)
  output_ = node.advertise<sensor_interface_msgs::Pointcloud2>(points_topic_array, 10);
  pubCompensation =
      node.advertise<std_msgs::Bool>("/sensor/lidar/robosense/using_compensation", 10);
  srv_ = boost::make_shared<dynamic_reconfigure::Server<TransformNodeCfg>>(
      private_nh);
  dynamic_reconfigure::Server<TransformNodeCfg>::CallbackType f;
  f = boost::bind(&Transform::reconfigure_callback, this, _1, _2);
  srv_->setCallback(f);

  sub_for_stamp_ = node.subscribe<sensor_interface_msgs::LidarScan>(
      packet_topic, 1, &Transform::stamp_checker, this);

  // subscribe to LidarScan packets using transform filter
  lidar_scan_.subscribe(node, packet_topic, 10);

  tf_filter_ = new tf::MessageFilter<sensor_interface_msgs::LidarScan>(lidar_scan_, listener_, config_.frame_id, 10);
  tf_filter_->registerCallback(boost::bind(&Transform::processScan, this, _1));

  // Diagnostics
  diagnostics_.setHardwareID("Robosense Transform");
  // Arbitrary frequencies since we don't know which RPM is used, and are only
  // concerned about monitoring the frequency.
  diag_min_freq_ = 2.0;
  diag_max_freq_ = 20.0;
  using namespace diagnostic_updater;
}

void Transform::reconfigure_callback(
    robosense_pointcloud::TransformNodeConfig &config, uint32_t level) {
  data_->setParameters(config.view_direction, config.view_width);
  config_.frame_id = tf::resolve(tf_prefix_, config.frame_id);
  ROS_INFO_STREAM("Fixed frame ID: " << config_.frame_id);
  LOG(INFO) << "Fixed frame ID: " << config_.frame_id;
  MLOG_INFO("Fixed Frame ID:%s", config.frame_id.c_str());
}

void Transform::stamp_checker(
    const sensor_interface_msgs::LidarScan::ConstPtr &msg) {
  if (msg->header.stamp < prev_stamp_) {
    ROS_WARN_STREAM("Detected jump back in time of "
                    << (prev_stamp_ - msg->header.stamp).toSec()
                    << "s. Clearing TF buffer.");
    LOG(WARNING) << "Detected jump back in time of "
                 << (prev_stamp_ - msg->header.stamp).toSec()
                 << "s. Clearing TF buffer.";
    MLOG_ERROR("Detected jump back in time of:%f s, Clearing TF buffer", (prev_stamp_ - msg->header.stamp).toSec());

    listener_.clear();
    prev_stamp_ = msg->header.stamp;
    return;
  }
  prev_stamp_ = msg->header.stamp;
}

/** @brief Callback for raw scan messages.
 *
 *  @pre TF message filter has already waited until the transform to
 *       the configured @c frame_id can succeed.
 */
void Transform::processScan(
    const sensor_interface_msgs::LidarScan::ConstPtr &scanMsg) {
  // long long t_begin = get_timenow();

  outMsg->points.clear();
  outMsg->points.resize(width_ * height_);
  outMsg->header.frame_id = frame_id_base_;
  outMsg->height = height_;
  outMsg->width = width_;

  std_msgs::Bool using_compensation;
  using_compensation.data = true;

  if (fixed_frame_id_ == frame_id_base_) {
    using_compensation.data = false;
  }

  static ros::Time history_stamp;
  if (history_stamp >= scanMsg->header.stamp) {
    ROS_ERROR_STREAM("Receive a history msg :" << scanMsg->header.stamp
                                               << ". The latest timestamp: "
                                               << history_stamp);
    LOG(ERROR) << "Receive a history msg :" << scanMsg->header.stamp
               << ". The latest timestamp: " << history_stamp;
    MLOG_ERROR("Receive a history msg: %ld.%06ld, the latest timestamp:%ld.%06ld", scanMsg->header.stamp.sec, scanMsg->header.stamp.nsec/1000, history_stamp.sec, history_stamp.nsec/1000);

    history_stamp = scanMsg->header.stamp;
    using_compensation.data = false;
  }
  if((scanMsg->header.stamp - history_stamp).toSec() > 0.2)
  {
    ROS_ERROR_STREAM("Robosense Drop Frames"<<"CurTime:"<<scanMsg->header.stamp << "LastTime:"<<history_stamp);
    MLOG_ERROR("Robosense Multi Scans Drop Frame, Curtime: %ld.%06ld.. LastTime:%ld.%06ld", scanMsg->header.stamp.sec, scanMsg->header.stamp.nsec/1000, history_stamp.sec, history_stamp.nsec/1000);
  }

  history_stamp = scanMsg->header.stamp;

  tf::StampedTransform tf_car2lidar;
  Eigen::Isometry3d is3d_car2lidar;
  try {
    listener_.lookupTransform(
        frame_id_base_,           // default [car]
        scanMsg->header.frame_id, // default [robosense_center]
        ros::Time(0), tf_car2lidar);
    is3d_car2lidar = TF2Eigen(tf_car2lidar);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("Can not lookup transform from lidar to car. [Skip current "
              "frame] . %s",
              ex.what());
    LOG(ERROR)
        << "Can not lookup transform from lidar to car. [Skip current frame]";
    MLOG_ERROR("Can not lookup transform from lidar to car. [Skip current frame]");
    return;
  }

  tf::StampedTransform tf_car2fixframe;
  Eigen::Isometry3d is3d_car2fixframe;
  if (using_compensation.data) {
    try {
      listener_.lookupTransform(
          frame_id_base_,  // default [car]
          fixed_frame_id_, // default enu [car or enu]
          scanMsg->header.stamp + motion_compensate_offset_, tf_car2fixframe);
      is3d_car2fixframe = TF2Eigen(tf_car2fixframe);
    } catch (tf::TransformException &ex) {
      using_compensation.data = false;
      ROS_WARN("Can not lookup transform from car to enu. [Close motion "
               "compensation] .%s",
               ex.what());
      LOG(WARNING) << "Can not lookup transform from car to enu. [Close motion "
                      "compensation]";
      MLOG_WARN("Can not lookup transform from car to enu. [Close motion compensation]");
    }
  }

  static int max_packet = 0;
  if(scanMsg->available == scanMsg->LIDAR_TYPE_ROBOSENSE){
    max_packet = scanMsg->robosense_packets.size() > max_packet ? scanMsg->robosense_packets.size():max_packet;
    if (max_packet > scanMsg->robosense_packets.size() &&
        max_packet - scanMsg->robosense_packets.size() > max_packet * 0.04) {
      LOG(ERROR) << "Lost udp packets, only recieve " << scanMsg->robosense_packets.size()
        << " Lost: " << max_packet - scanMsg->robosense_packets.size();
      ROS_WARN_STREAM("Lost udp packets, only recieve "
          << scanMsg->robosense_packets.size());
      MLOG_ERROR("Lost udp packets, only recv:%d", scanMsg->robosense_packets.size());
    } else {
      LOG(INFO) << "Receive scan packets' size: " << scanMsg->robosense_packets.size();
      ROS_INFO_STREAM("Receive scan packets' size: " << scanMsg->robosense_packets.size());
      MLOG_INFO("Receive scan packets' size: %d", scanMsg->robosense_packets.size());
    }

    // process each packet provided by the driver
    for (size_t next = 0; next < scanMsg->robosense_packets.size(); ++next) {
      tf::StampedTransform transform;
      Eigen::Isometry3d is3d_current;
      ros::Time tmp_time;
      if (using_compensation.data) {
        try {
          listener_.lookupTransform(fixed_frame_id_, // default enu [car or enu]
              frame_id_base_,  // default [car]
              tmp_time.fromNSec(scanMsg->robosense_packets[next].stamp), transform);
          is3d_current = TF2Eigen(transform);
          update_stamptf(transform);
        } catch (tf::TransformException &ex) {
          // no tf from [fixed_frame_id_] to [frame_id_base_]
          if (!infer_tf(tmp_time.fromNSec(scanMsg->robosense_packets[next].stamp), is3d_current)) {
            using_compensation.data = false;
            ROS_DEBUG_STREAM("Infer_tf error in packet:" << next);
            LOG(ERROR) << "Infer_tf error in packet:" << next
              << " [Close motion compensation]";
          }
        }
      }
      if (using_compensation.data) {
        Eigen::Isometry3d is3d_compensation =
          is3d_car2fixframe * is3d_current * is3d_car2lidar;
        data_->unpack_rsruby128(scanMsg->robosense_packets[next], *outMsg, is3d_compensation);
      } else {
        data_->unpack_rsruby128(scanMsg->robosense_packets[next], *outMsg, is3d_car2lidar);
      }
    } // end for
  }

  // publish robosense_points
  if (outMsg->points.size()) {
    pcl_conversions::toPCL(scanMsg->header.stamp + motion_compensate_offset_,
                           outMsg->header.stamp);
    output_.publish(outMsg);
    pubCompensation.publish(using_compensation);
    data_->reset_startingangle();
    ros::Duration current_delay =
        ros::Time::now() - scanMsg->header.stamp - motion_compensate_offset_;
    LOG(INFO) << "Robosense_points delay: " << current_delay;
    ROS_INFO_STREAM("Robosense_points delay: " << current_delay);
  }
}

// update tf
bool Transform::update_stamptf(const tf::StampedTransform &stampedtransform) {
  que_mutex_.lock();
  bool flag = false;
  Eigen::Vector3d vec;
  tf::vectorTFToEigen(stampedtransform.getOrigin(), vec);
  Stamp_tf stf;
  stf.is3d = TF2Eigen(stampedtransform);
  stf.stamp = stampedtransform.stamp_;
  stf.vec = vec;

  if (!tf_que_.empty() && stf.stamp == tf_que_.back().stamp) {
    que_mutex_.unlock();
    return flag;
  }

  if (!tf_que_.empty() && stf.stamp < tf_que_.back().stamp) {
    ROS_ERROR_STREAM("Time stamp backwards !");
    MLOG_ERROR("Timestamp backwards");
    tf_que_.clear();
    tf_que_.push_back(stf);
    flag = true;
  } else if (tf_que_.empty()) {
    tf_que_.push_back(stf);
    flag = true;
  } else if (stf.vec != tf_que_.back().vec) {
    tf_que_.push_back(stf);

    Eigen::Vector3f t;
    Eigen::Vector3d pos(stf.is3d.translation());
    t = pos.cast<float>();
    flag = true;
  }
  while (tf_que_.size() > max_queuesize_) {
    tf_que_.pop_front();
  }
  que_mutex_.unlock();
  return flag;
}

bool Transform::infer_tf(const ros::Time stamp, Eigen::Isometry3d &trans) {
  bool tf_found = false;
  que_mutex_.lock();
  if (tf_que_.empty()) {
    ROS_WARN_STREAM("tf_que_ is empty.");
  } else {
    auto seek =
        std::lower_bound(tf_que_.begin(), tf_que_.end(), stamp,
                         [&](const Stamp_tf tf_stamp, const ros::Time &stamp) {
                           return tf_stamp.stamp < stamp;
                         });
    if (seek == tf_que_.begin()) {
      ROS_WARN_STREAM("tf_que_"
                      << " search time is too old, current/oldest/newest: "
                      << std::fixed << stamp << " / " << (*seek).stamp << " / "
                      << (*(tf_que_.end() - 1)).stamp);
    } else {
      Eigen::Isometry3d T1, T2;
      double t0, t1, t2;
      if (seek == tf_que_.end()) {
        auto &p1 = *(seek - 2);
        auto &p2 = *(seek - 1);
        T1 = p1.is3d;
        T2 = p2.is3d;

        t0 = stamp.toSec();
        t1 = p1.stamp.toSec();
        t2 = p2.stamp.toSec();

      } else {
        auto &p1 = *(seek - 1);
        auto &p2 = *seek;

        T1 = p1.is3d;
        T2 = p2.is3d;

        t0 = stamp.toSec();
        t1 = p1.stamp.toSec();
        t2 = p2.stamp.toSec();
      }

      double scale = (t0 - t1) / (t2 - t1);
      if ((t2 - t1) > stamp_tolerance || (t2 - t1) < 0) {
        ROS_WARN_STREAM("tf_que_"
                        << " t2 - t1>tolerance:" << t2 - t1 << ">"
                        << stamp_tolerance << " cur/t1/t2:" << std::fixed << t0
                        << " " << t1 << " " << t2);
      } else if ((t0 - t1) > delay_tolerance || (t0 - t1) < 0) {
        ROS_WARN_STREAM("tf_que_"
                        << " t0 - t1>tolerance:" << t0 - t1 << ">"
                        << delay_tolerance << " cur/t0/t1:" << std::fixed << t0
                        << " " << t0 << " " << t1);
      } else {
        tf_found = true;
        trans = interpolation(T1, T2, scale);
      }
    }
  }
  que_mutex_.unlock();

  return tf_found;
}

} // namespace robosense_pointcloud
