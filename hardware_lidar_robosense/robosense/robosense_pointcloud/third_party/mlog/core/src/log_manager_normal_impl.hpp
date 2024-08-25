#pragma once

#include "background_dumper_impl.hpp"
#include "dumper_helper.hpp"
#include "mlog/file_helper.h"
#include "mlog/mlog.h"
#include <algorithm>
#include <cassert>
#include <cctype>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <inttypes.h>
#include <memory>
#include <mutex>

using namespace mlog;

namespace {

static const char *Mlog_get_log_name[] = {"DBG", "INF", "WRN", "ERR", "FATAL"};

inline mlog::MLogLevel get_mlog_level(std::string level) {
  std::transform(level.begin(), level.end(), level.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (level == "debug") {
    return mlog::MLogLevel::MLOG_LEVEL_DEBUG;
  } else if (level == "info") {
    return mlog::MLogLevel::MLOG_LEVEL_INFO;
  } else if (level == "warn") {
    return mlog::MLogLevel::MLOG_LEVEL_WARN;
  } else if (level == "error") {
    return mlog::MLogLevel::MLOG_LEVEL_ERROR;
  } else if (level == "fatal") {
    return mlog::MLogLevel::MLOG_LEVEL_FATAL;
  } else {
    // use default level
    return mlog::MLogLevel::MLOG_LEVEL_INFO;
  }
}

class MLogManagerNormalImpl : public MLogManager {
public:
  MLogManagerNormalImpl(MLogSettings settings) : settings_(settings) {
    constexpr auto MLOG_LEVEL = "MLOG_LEVEL";
    auto log_level_str = std::getenv(MLOG_LEVEL);
    if (log_level_str) {
      auto log_level = get_mlog_level(log_level_str);
      settings_.level = log_level;
    }
    if (settings.export_frequence > 0)
      stashed_duration_ns_ = 1e9 / settings.export_frequence;
    if (settings_.endpoint & MLOG_ENDPOINT_FILE) {
      make_recursive_directory(directory_name(settings_.log_file));
    }
    if (settings_.enable_background_dump) {
      background_dumper_ = BackgroundDumper::instance();
    }
    if (settings_.ros_settings.enable_ros_publisher_lazy_init == false &&
        (settings_.endpoint & MLOG_ENDPOINT_ROS_TOPIC)) {
      ros_endpoint_callback(
          {}, settings_.rostopic_name,
          settings_.ros_settings.ros_publisher_init_waiting_time_sec, true);
    }
    if (settings_.mfr_settings.enable_mfr_publisher_lazy_init == false &&
        (settings_.endpoint & MLOG_ENDPOINT_MFR_TOPIC)) {
      mfr_endpoint_callback(
          {}, settings_.mfrtopic_name,
          settings_.mfr_settings.mfr_publisher_init_waiting_time_sec, true,
          settings_.mfr_settings.node_handle);
    }
    file_helper_ = std::make_shared<MLogFileHelper>(settings_);
  }
  ~MLogManagerNormalImpl() { export_data_if_timestamp_meet(true); }

  void record_timestamp(std::string msg_id, std::string tag,
                        uint64_t timestamp_ns) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (settings_.endpoint == 0)
      return;
    if (timestamp_ns == 0) {
      timestamp_ns = system_now();
    }
    std::string buffer =
        msg_id + " " + tag + " " + std::to_string(timestamp_ns);
    entries_.push_back(MLogEntry{
        .type = MLogDataType::MLOG_DATA_TYPE_MSG_TIMESTAMP,
        .data = std::move(buffer),
        .timestamp_ns = steady_now(),
    });

    export_data_if_timestamp_meet();
  }

  void record_relation(std::string current_msg_id,
                       std::string relevant_msg_id) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (settings_.endpoint == 0)
      return;

    std::string buffer = current_msg_id + " " + relevant_msg_id;
    entries_.push_back(MLogEntry{
        .type = MLogDataType::MLOG_DATA_TYPE_MSG_RELATION,
        .data = std::move(buffer),
        .timestamp_ns = steady_now(),
    });
    export_data_if_timestamp_meet();
  }

  void record_log(MLogLevel level, const char *fmt, ...) {
    if (settings_.endpoint == 0) {
      return;
    }
    if (level < settings_.level) {
      return;
    }
    constexpr int HEAD_LEN = 1024;
    va_list list;
    va_start(list, fmt);
    auto log_content = va_arg_to_string(fmt, list);
    va_end(list);

    std::vector<char> buffer(log_content.size() + HEAD_LEN);
#if __WORDSIZE == 64
    snprintf(buffer.data(), buffer.size(), "[%s][%lu]:%s",
             Mlog_get_log_name[static_cast<int>(level)], steady_now(),
             log_content.c_str());
#else
#if defined(__QNX__)
    snprintf(buffer.data(), buffer.size(), "[%s][%" PRIu64 "]:%s",
             Mlog_get_log_name[static_cast<int>(level)], steady_now(),
             log_content.c_str());
#else
    snprintf(buffer.data(), buffer.size(), "[%s][%llu]:%s",
             Mlog_get_log_name[static_cast<int>(level)], steady_now(),
             log_content.c_str());
#endif
#endif
    std::unique_lock<std::mutex> lock(mutex_);
    entries_.push_back(MLogEntry{
        .type = MLogDataType::MLOG_DATA_TYPE_NORMAL,
        .data = buffer.data(),
        .timestamp_ns = steady_now(),
    });

    if (settings_.export_type == MLOG_EXPORT_TYPE_PERIODIC) {
      export_data_if_timestamp_meet(false);
    } else if (settings_.export_type == MLOG_EXPORT_TYPE_ON_FLUSH) {
      if (settings_.data_entry_limits > 0 &&
          static_cast<int>(entries_.size()) >= settings_.data_entry_limits) {
        export_data_if_timestamp_meet(true);
      }
    }

    if (level == MLOG_LEVEL_FATAL) {
      std::abort();
    }
  }

  void flush_log() {
    std::unique_lock<std::mutex> lock(mutex_);
    export_data_if_timestamp_meet(true);
  }

private:
  inline uint64_t steady_now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }

  inline uint64_t system_now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }

  std::string va_arg_to_string(const char *fmt, va_list list) {
    va_list list_copy1, list_copy2;
    va_copy(list_copy1, list);
    va_copy(list_copy2, list);
    size_t actual_size = std::vsnprintf(nullptr, 0, fmt, list_copy1);
    std::vector<char> buf(1 + actual_size);
    va_end(list_copy1);

    size_t valid_size = vsnprintf(buf.data(), buf.size(), fmt, list_copy2);
    va_end(list_copy2);

    assert(valid_size == actual_size);
    return std::string(buf.begin(), buf.begin() + valid_size);
  }

  inline void export_data_if_timestamp_meet(bool force_export = false) {
    if (entries_.empty()) {
      return;
    }
    auto steady_timestamp_ns = steady_now();
    if (stashed_duration_ns_ == 0 ||
        steady_timestamp_ns - last_record_timestamp_ns_ >
            stashed_duration_ns_ ||
        force_export) {
      last_record_timestamp_ns_ = steady_timestamp_ns;

      LogDumpRequest request{};
      request.endpoint = settings_.endpoint;
      request.endpoint_callback = settings_.endpoint_callback;
      request.file_helper = file_helper_;
      request.rostopic_name = settings_.rostopic_name;
      request.mfrtopic_name = settings_.mfrtopic_name;
      request.log_entries = entries_;
      request.ros_setttings = settings_.ros_settings;
      request.mfr_setttings = settings_.mfr_settings;
      entries_.clear();

      if (settings_.enable_background_dump) {
        background_dumper_->get_buffer().push(request);
      } else {
        dump_log_data(request);
      }
    }
  }

  std::shared_ptr<MLogFileHelper> file_helper_;
  MLogSettings settings_;
  std::vector<MLogEntry> entries_;
  std::mutex mutex_;
  uint64_t last_record_timestamp_ns_ = 0;
  uint64_t stashed_duration_ns_ = 0;
  std::shared_ptr<BackgroundDumper> background_dumper_;
};

} // namespace
