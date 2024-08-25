#pragma once

#include "mlog_msg_id/mlog_msg_id.hpp"
#include "profiling.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace mlog {
enum MLogDataType {
  MLOG_DATA_TYPE_NORMAL,
  MLOG_DATA_TYPE_MSG_TIMESTAMP,
  MLOG_DATA_TYPE_MSG_RELATION,
};

enum MLogLevel {
  MLOG_LEVEL_DEBUG = 0,
  MLOG_LEVEL_INFO,
  MLOG_LEVEL_WARN,
  MLOG_LEVEL_ERROR,
  MLOG_LEVEL_FATAL,
};

struct MLogEntry {
  MLogDataType type;
  std::string data;
  uint64_t timestamp_ns;
};

enum MLogEndpoint {
  MLOG_ENDPOINT_STDERR = 1 << 0,
  MLOG_ENDPOINT_CUSTOM_CALLBACK = 1 << 1,
  MLOG_ENDPOINT_ROS_TOPIC = 1 << 2,
  MLOG_ENDPOINT_FILE = 1 << 3,
  MLOG_ENDPOINT_MFR_TOPIC = 1 << 4,
};

enum MLogFileEndpointType {
  MLOG_FILE_ENDPOINT_TYPE_APPEND = 0,
  MLOG_FILE_ENDPOINT_TYPE_WRITE = 1,
};

enum MLogExportType {
  MLOG_EXPORT_TYPE_PERIODIC = 0,
  MLOG_EXPORT_TYPE_ON_FLUSH = 1,
};

typedef void MLogEndpointCallback(const std::vector<MLogEntry> &data);

struct MLogRosSettings {
  bool enable_ros_publisher_lazy_init = true;
  double ros_publisher_init_waiting_time_sec = 0;
};

struct MLogMfrSettings {
  bool enable_mfr_publisher_lazy_init = true;
  double mfr_publisher_init_waiting_time_sec = 0;
  void *node_handle = nullptr;
};

struct MLogSettings {
  int data_entry_limits = 10000;
  int export_frequence = 1;
  size_t endpoint = MLOG_ENDPOINT_STDERR;
  MLogEndpointCallback *endpoint_callback = nullptr;
  std::string extra;
  MLogLevel level = MLogLevel::MLOG_LEVEL_INFO;
  std::string log_file;
  MLogFileEndpointType file_endpoint_type;
  std::string rostopic_name;
  std::string mfrtopic_name;
  bool enable_background_dump;
  MLogExportType export_type = MLogExportType::MLOG_EXPORT_TYPE_PERIODIC;
  MLogRosSettings ros_settings;
  MLogMfrSettings mfr_settings;
  uint32_t max_file_size = 0;
  uint32_t max_file_num = 1;
};

class MLogManager {
public:
  virtual ~MLogManager() = default;
  virtual void record_timestamp(std::string msg_id, std::string tag,
                                uint64_t timestamp_ns) = 0;
  virtual void record_relation(std::string current_msg_id,
                               std::string relevant_msg_id) = 0;
  virtual void record_log(MLogLevel level, const char *fmt, ...) = 0;
  virtual void flush_log() = 0;

  static std::shared_ptr<MLogManager> make();
  static std::shared_ptr<MLogManager> make(const MLogSettings &settings);

  static void set_settings(MLogSettings settings);
  static MLogSettings &get_settings();

private:
  static MLogSettings settings_;
};

inline void MLog_set_endpoint(MLogEndpoint endpoint,
                              MLogEndpointCallback *callback = nullptr) {
  MLogManager::get_settings().endpoint = endpoint;
  MLogManager::get_settings().endpoint_callback = callback;
}

#define MLOG_RECORD_TIMESTAMP(msg_id, tag, timestamp_ns)                       \
  do {                                                                         \
    static auto manager = mlog::MLogManager::make();                           \
    manager->record_timestamp(msg_id, tag, timestamp_ns);                      \
  } while (false)

#define MLOG_RECORD_RELATION(current_msg_id, relevant_msg_id)                  \
  do {                                                                         \
    static auto manager = mlog::MLogManager::make();                           \
    manager->record_relation(current_msg_id, relevant_msg_id);                 \
  } while (false)

#define MLOG_RECORD_LOG(level, ...)                                            \
  do {                                                                         \
    static auto manager = mlog::MLogManager::make();                           \
    manager->record_log(level, __VA_ARGS__);                                   \
  } while (false)

#define MLOG_DEBUG(...) MLOG_RECORD_LOG(mlog::MLOG_LEVEL_DEBUG, __VA_ARGS__)
#define MLOG_INFO(...) MLOG_RECORD_LOG(mlog::MLOG_LEVEL_INFO, __VA_ARGS__)
#define MLOG_WARN(...) MLOG_RECORD_LOG(mlog::MLOG_LEVEL_WARN, __VA_ARGS__)
#define MLOG_ERROR(...) MLOG_RECORD_LOG(mlog::MLOG_LEVEL_ERROR, __VA_ARGS__)
#define MLOG_FATAL(...) MLOG_RECORD_LOG(mlog::MLOG_LEVEL_FATAL, __VA_ARGS__)

#define MLOG_FLUSH()                                                           \
  do {                                                                         \
    static auto manager = mlog::MLogManager::make();                           \
    manager->flush_log();                                                      \
  } while (false)

} // namespace mlog
