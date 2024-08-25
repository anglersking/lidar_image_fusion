#include <cstdint>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include "mlog/mlog.h"
#include <chrono>
#include <mutex>
#include <queue>
#include <thread>

#ifdef MLOG_ENABLE_MFR
int main(int argc, char *argv[]) {
  mlog::MLog_set_endpoint(mlog::MLogEndpoint::MLOG_ENDPOINT_MFR_TOPIC);
  auto settings = mlog::MLogManager::get_settings();
  mlog::MLogSettings new_settings(settings);
  new_settings.endpoint =
      mlog::MLOG_ENDPOINT_FILE | mlog::MLOG_ENDPOINT_MFR_TOPIC;
  new_settings.mfrtopic_name = "/mlog";
  new_settings.level = mlog::MLOG_LEVEL_INFO;
  new_settings.log_file = "log1.txt";
  new_settings.mfr_settings.mfr_publisher_init_waiting_time_sec = 1;
  mlog::MLogManager::set_settings(new_settings);

  MLOG_DEBUG("debug"); // this line will not print out because of log level
  MLOG_INFO("info");
  MLOG_WARN("warn");
  MLOG_ERROR("error");
}
#else
int main(int argc, char *argv[]) {
  printf("Not deffine MLOG_ENABLE_MFR.\n");
  abort();
}
#endif