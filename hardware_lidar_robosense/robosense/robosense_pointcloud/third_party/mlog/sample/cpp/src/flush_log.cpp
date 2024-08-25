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

  /* If export_type has been set to 'MLOG_EXPORT_TYPE_ON_FLUSH', logs will not
     be exported until flush interface has been called;
     Otherwise, logs will be exported periodically according to the frequency
     in the settings */
  new_settings.export_type = mlog::MLOG_EXPORT_TYPE_ON_FLUSH;
  mlog::MLogManager::set_settings(new_settings);

  MLOG_INFO("info");
  MLOG_WARN("warn");
  MLOG_ERROR("error");
  MLOG_FLUSH();
}
