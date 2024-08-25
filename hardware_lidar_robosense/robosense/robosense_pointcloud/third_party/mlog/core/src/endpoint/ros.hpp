#pragma once

#include "mlog/mlog.h"
#include <cstdlib>
#include <string>
#include <vector>

#ifdef MLOG_ENABLE_ROS
#include "mlog_msgs/Logs.h" //ROS message header
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <chrono>
#include <thread>

namespace {
inline mlog_msgs::Entry convert(const mlog::MLogEntry &entry) {

  static_assert(static_cast<int>(mlog_msgs::Entry::DATA_TYPE_NORMAL) ==
                    mlog::MLogDataType::MLOG_DATA_TYPE_NORMAL,
                "DATA_TYPE_NORMAL mismatch");
  static_assert(static_cast<int>(mlog_msgs::Entry::DATA_TYPE_MSG_TIMESTAMP) ==
                    mlog::MLogDataType::MLOG_DATA_TYPE_MSG_TIMESTAMP,
                "DATA_TYPE_MSG_TIMESTAMP mismatch");
  static_assert(static_cast<int>(mlog_msgs::Entry::DATA_TYPE_MSG_RELATION) ==
                    mlog::MLogDataType::MLOG_DATA_TYPE_MSG_RELATION,
                "DATA_TYPE_MSG_RELATION mismatch");
  mlog_msgs::Entry ret{};
  ret.data = entry.data;
  ret.type = entry.type;
  return ret;
}

inline void ros_endpoint_callback(const std::vector<mlog::MLogEntry> &entries,
                                  const std::string &rostopic_name,
                                  double init_waiting_time_sec,
                                  bool only_init) {
  static std::shared_ptr<ros::Publisher> publisher = nullptr;
#ifdef MLOG_ENABLE_MDC_ROS
  if (!ros::IsInitialized())
#else
  if (!ros::isInitialized())
#endif
    return;
  if (publisher == nullptr) {
    constexpr auto MLOG_TOPIC = "mlog_topic";
    ros::NodeHandle n;
    std::string mlog_topic;
    if (rostopic_name.empty())
      n.param<std::string>(MLOG_TOPIC, mlog_topic, "/mlog/logs");
    else
      mlog_topic = rostopic_name;
#ifdef MLOG_ENABLE_MDC_ROS
    publisher = std::make_shared<ros::Publisher>(
        n.Advertise<mlog_msgs::Logs>(mlog_topic, 10, 300000));
#else
    publisher = std::make_shared<ros::Publisher>(
        n.advertise<mlog_msgs::Logs>(mlog_topic, 10));
#endif
    ROS_INFO("%s: %s", MLOG_TOPIC, mlog_topic.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(init_waiting_time_sec * 1000)));
  }
  if (only_init)
    return;
  if (
#ifdef MLOG_ENABLE_MDC_ROS
      ros::Ok()
#else
      ros::ok()
#endif
  ) {
    mlog_msgs::Logs logs{};
    logs.header.stamp = ros::Time::now();
    for (const auto &entry : entries) {
      logs.entries.push_back(convert(entry));
    }
#ifdef MLOG_ENABLE_MDC_ROS
    publisher->Publish(logs);
#else
    publisher->publish(logs);
#endif
  }
}
} // namespace

#else

namespace {
inline void ros_endpoint_callback(const std::vector<mlog::MLogEntry> &entries,
                                  const std::string &rostopic_name,
                                  double init_waiting_time_sec,
                                  bool only_init) {
  fprintf(stderr, "ROS Endpoint is invalid\n");
  std::abort();
}
} // namespace

#endif
