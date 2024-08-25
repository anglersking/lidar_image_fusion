#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#define MLOG_MERGE_SUB(X, Y) X##Y
#define MLOG_MERGE(X, Y) MLOG_MERGE_SUB(X, Y)

#ifdef MLOG_ENABLE_PROFILING
#define MLOG_PROFILING(name)                                                   \
  mlog::TimeMeasurer MLOG_MERGE(MEASURE_TIME_, __LINE__)(name);
#else
#define MLOG_PROFILING(name)
#endif

#ifdef __QNX__
#include <stdlib.h>
#endif

#if defined(_MSC_VER)
#include <process.h>
#include <stdio.h>
namespace {
inline int64_t current_process_id() { return _getpid()(); }
inline int64_t current_thread_id() {
  return std::hash<unsigned int>{}(
      std::hash<std::thread::id>{}(std::this_thread::get_id()));
}
} // namespace
#else
#include <unistd.h>
namespace {
inline int64_t current_process_id() { return getpid(); }
inline int64_t current_thread_id() { return pthread_self(); }
} // namespace
#endif

namespace mlog {
class TimeMeasurer {
public:
  TimeMeasurer(const char *name) : name_(name) { log_time("B"); }
  ~TimeMeasurer() { log_time("E"); }

private:
  inline void log_time(const char *phase) {
    static auto output_file = []() -> std::shared_ptr<FILE> {
#ifdef __QNX__
      auto profile_path = getenv("MLOG_PROFILING_PATH");
#else
      auto profile_path = std::getenv("MLOG_PROFILING_PATH");
#endif
      std::string profile_path_suffix = "";
      { // append suffix
#ifdef __QNX__
        auto suffix_environment = getenv("MLOG_PROFILING_PATH_SUFFIX");
#else
        auto suffix_environment = std::getenv("MLOG_PROFILING_PATH_SUFFIX");
#endif
        if (suffix_environment == nullptr) {
          auto current_timestamp_us =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();
          profile_path_suffix = "." + std::to_string(current_timestamp_us) +
                                "." + std::to_string(current_process_id());
        } else {
          profile_path_suffix = suffix_environment;
        }
      }

      if (profile_path == nullptr) { // disable output
        return {};
      } else if (std::string(profile_path) == "") { // output to stdout
        fprintf(stdout, "[");
        return std::shared_ptr<FILE>(stdout, [](FILE *) {});
      } else { // output to customized file
        auto p_file = fopen((profile_path + profile_path_suffix).c_str(), "w");
        if (p_file == nullptr)
          return nullptr;
        fprintf(p_file, "[");
        return std::shared_ptr<FILE>(p_file,
                                     [](FILE *p_file) { fclose(p_file); });
      }
    }();

    static thread_local int64_t thread_id = current_thread_id();
    static thread_local int64_t process_id = current_process_id();

    if (output_file) {
      fprintf(output_file.get(),
              "{\"ts\": %ld, \"pid\":%ld, \"tid\":%ld, \"ph\":\"%s\", "
              "\"name\":\"%s\"},\n",
              static_cast<int64_t>(
                  std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count()),
              process_id, thread_id, phase, name_);
    }
  }
  const char *name_;
};

} // namespace mlog
