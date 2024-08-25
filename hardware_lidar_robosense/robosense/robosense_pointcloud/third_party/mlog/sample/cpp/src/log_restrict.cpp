#include <cstdint>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include "mlog/mlog.h"
#include <chrono>
#include <mutex>
#include <queue>
#include <thread>

uint64_t now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int main(int argc, char *argv[]) {
  auto settings = mlog::MLogManager::get_settings();
  mlog::MLogSettings new_settings(settings);
  new_settings.endpoint = mlog::MLOG_ENDPOINT_FILE | mlog::MLOG_ENDPOINT_STDERR;
  new_settings.level = mlog::MLOG_LEVEL_INFO;
  new_settings.log_file = "log1.txt";
  new_settings.max_file_num = 3;
  new_settings.max_file_size = 100;
  new_settings.export_frequence = 0;
  new_settings.file_endpoint_type = mlog::MLOG_FILE_ENDPOINT_TYPE_WRITE;
  mlog::MLogManager::set_settings(new_settings);

  static auto manager = mlog::MLogManager::make();
  manager->record_log(mlog::MLOG_LEVEL_DEBUG, "debug");
  manager->record_log(mlog::MLOG_LEVEL_INFO, "info");
  manager->record_log(mlog::MLOG_LEVEL_WARN, "warn");
  manager->record_log(mlog::MLOG_LEVEL_ERROR, "error");
  manager->record_timestamp("msg_id", "recv", now());
  manager->record_relation("result_id", "msg_id");

  std::mutex mutex;
  std::thread record_relation([]() {
    for (int i = 0; i < 10; ++i) {
      std::string msg_id = std::string("msg") + std::to_string(i);
      manager->record_timestamp(msg_id, "recv", now());
      manager->record_relation("result_id", msg_id);
    }
  });

  std::thread record_log([]() {
    manager->record_log(mlog::MLOG_LEVEL_INFO, "fatal");
    manager->record_log(mlog::MLOG_LEVEL_INFO, "fatal");
    manager->record_log(mlog::MLOG_LEVEL_INFO, "fatal");
    manager->record_log(mlog::MLOG_LEVEL_INFO, "fatal");
  });

  record_relation.join();
  record_log.join();
}
