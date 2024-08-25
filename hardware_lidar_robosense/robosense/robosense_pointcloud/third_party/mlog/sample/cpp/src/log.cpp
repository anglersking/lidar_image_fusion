#include <cstdint>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include "mlog/mlog.h"
#include <chrono>
#include <mutex>
#include <queue>
#include <thread>

int main(int argc, char *argv[]) {
  auto settings = mlog::MLogManager::get_settings();
  mlog::MLogSettings new_settings(settings);
  new_settings.endpoint = mlog::MLOG_ENDPOINT_FILE | mlog::MLOG_ENDPOINT_STDERR;
  new_settings.level = mlog::MLOG_LEVEL_INFO;
  new_settings.log_file = "log1.txt";
  mlog::MLogManager::set_settings(new_settings);

  MLOG_DEBUG("debug"); // this line will not print out because of log level
  MLOG_INFO("info");
  MLOG_WARN("warn");
  MLOG_ERROR("error");

  mlog::MLogManager::set_settings(settings);
  new_settings.log_file = "log2.txt";
  new_settings.endpoint = mlog::MLOG_ENDPOINT_FILE;
  new_settings.level = mlog::MLOG_LEVEL_DEBUG;
  mlog::MLogManager::set_settings(new_settings);
  MLOG_DEBUG("2.debug");
  MLOG_INFO("2.info");
  mlog::MLogManager::set_settings(settings);
}
