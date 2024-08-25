#pragma once
#include "endpoint/file.hpp"
#include "endpoint/mfr.hpp"
#include "endpoint/ros.hpp"
#include "endpoint/stderr.hpp"
#include "mlog/file_helper.h"
#include "mlog/mlog.h"

namespace mlog {
namespace {

struct LogDumpRequest {
  size_t endpoint;
  MLogEndpointCallback *endpoint_callback;
  std::shared_ptr<MLogFileHelper> file_helper;
  std::string rostopic_name;
  std::string mfrtopic_name;
  std::vector<MLogEntry> log_entries;
  MLogRosSettings ros_setttings;
  MLogMfrSettings mfr_setttings;
};

void dump_log_data(const LogDumpRequest &request) {
  if (request.endpoint & MLOG_ENDPOINT_STDERR) {
    stderr_endpoint_callback(request.log_entries);
  }
  if (request.endpoint & MLOG_ENDPOINT_ROS_TOPIC) {
    ros_endpoint_callback(
        request.log_entries, request.rostopic_name,
        request.ros_setttings.ros_publisher_init_waiting_time_sec, false);
  }
  if (request.endpoint & MLOG_ENDPOINT_CUSTOM_CALLBACK) {
    if (request.endpoint_callback != nullptr) {
      request.endpoint_callback(request.log_entries);
    }
  }
  if (request.endpoint & MLOG_ENDPOINT_FILE) {
    file_endpoint_callback(request.log_entries, request.file_helper);
  }
  if (request.endpoint & MLOG_ENDPOINT_MFR_TOPIC) {
    mfr_endpoint_callback(
        request.log_entries, request.mfrtopic_name,
        request.mfr_setttings.mfr_publisher_init_waiting_time_sec, false,
        request.mfr_setttings.node_handle);
  }
}
} // namespace
} // namespace mlog
