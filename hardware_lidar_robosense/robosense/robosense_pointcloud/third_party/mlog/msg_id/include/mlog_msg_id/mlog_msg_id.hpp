#pragma once

#include <string>

#if defined(MLOG_ENABLE_ROS) && !defined(MLOG_ENABLE_MDC_ROS)
#include <ros/time.h>
#include <std_msgs/Header.h>
#endif

namespace mlog {

inline std::string _concat(std::initializer_list<std::string> modules) {
  std::string ret;
  for (const std::string &module : modules) {
    ret.append("-");
    ret.append(module);
  }
  return ret;
}

template <typename... T>
std::string MLOG_msg_id_ros(uint64_t timestamp_ns, T &&... modules) {
  return std::to_string(timestamp_ns) + _concat({std::forward<T>(modules)...});
}

#if defined(MLOG_ENABLE_ROS) && !defined(MLOG_ENABLE_MDC_ROS)
template <typename... T>
std::string MLOG_msg_id_ros(ros::Time time, T &&... modules) {
  return std::to_string(time.toNSec()) + _concat({std::forward<T>(modules)...});
}

template <typename... T>
std::string MLOG_msg_id_ros(const std_msgs::Header &header, T &&... modules) {
  return std::to_string(header.stamp.toNSec()) +
         _concat({std::forward<T>(modules)...});
}
#endif

} // namespace mlog
