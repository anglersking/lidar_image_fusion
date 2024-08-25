#pragma once
#include "mlog.h"
#include <mutex>
#include <queue>
#include <string>

namespace mlog {

class MLogFileHelper {
private:
  inline std::string now() {
    return std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
  }

  inline void max_file_restrict() {
    if (max_file_size_ == 0)
      return;

    size_t position = file_name_.find_last_of(".");
    std::string log_file_info;
    if (position == std::string::npos) {
      log_file_info = "";
    } else {
      log_file_info = file_name_.substr(0, position + 1);
    }

    // max_file_size
    struct stat log_feature;
    if (get_file_info(file_name_, log_feature)) { // Only the first time getting
                                                  // file will be false(file
                                                  // does not exist).
      if (static_cast<uint32_t>(log_feature.st_size) >= max_file_size_) {
        std::string new_file = log_file_info + now();
        file_name_ = new_file;
        file_list_.push(file_name_);
        if (file_endpoint_type_settings_ == MLOG_FILE_ENDPOINT_TYPE_WRITE) {
          file_endpoint_type_ = MLOG_FILE_ENDPOINT_TYPE_WRITE;
        }
        while (static_cast<uint32_t>(file_list_.size()) > max_file_num_) {
          std::string file_oldest = file_list_.front();
          delete_file(file_oldest);
          file_list_.pop();
        }
      }
    }
  }

public:
  MLogFileHelper(MLogSettings settings) {
    if (settings.max_file_size == 0) {
      file_name_ = settings.log_file;
    } else {
      file_name_ = settings.log_file + "." + now();
    }
    file_list_.push(file_name_);
    file_endpoint_type_ = settings.file_endpoint_type;
    file_endpoint_type_settings_ = settings.file_endpoint_type;
    max_file_size_ = settings.max_file_size;
    max_file_num_ = settings.max_file_num;
  }
  ~MLogFileHelper() = default;

  void write_log(const std::vector<MLogEntry> &entries) {
    std::unique_lock<std::mutex> lock(mutex_);
    max_file_restrict();
    FILE *file = nullptr;
    if (file_endpoint_type_ == MLOG_FILE_ENDPOINT_TYPE_WRITE) {
      file = fopen(file_name_.c_str(), "w+");
    } else {
      file = fopen(file_name_.c_str(), "a+");
    }

    // for auto close
    std::shared_ptr<FILE> p_file(file, [](FILE *file) { fclose(file); });
    for (const auto &entry : entries) {
      fprintf(file, "%s %s\n", get_data_type_name(entry.type),
              entry.data.c_str());
    }

    if (file_endpoint_type_ == MLOG_FILE_ENDPOINT_TYPE_WRITE) {
      file_endpoint_type_ = MLOG_FILE_ENDPOINT_TYPE_APPEND;
    }
  }

private:
  std::queue<std::string> file_list_;
  std::string file_name_;
  MLogFileEndpointType file_endpoint_type_;
  MLogFileEndpointType file_endpoint_type_settings_;
  uint32_t max_file_size_;
  uint32_t max_file_num_;
  std::mutex mutex_;
};

} // namespace mlog