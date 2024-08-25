#include <cstdint>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#include "mlog/mlog.h"
#include <chrono>
#include <mutex>
#include <queue>
#include <thread>

#ifdef MLOG_ENABLE_ROS
#include "ros/ros.h"

uint64_t now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_mlog");
  mlog::MLog_set_endpoint(mlog::MLogEndpoint::MLOG_ENDPOINT_ROS_TOPIC);
  auto &settings = mlog::MLogManager::get_settings();
  settings.rostopic_name = "/mlog/mylogs";
  settings.level = mlog::MLogLevel::MLOG_LEVEL_DEBUG;
  while (1) {
    static int count = 0;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    MLOG_INFO("info");
    MLOG_DEBUG("debug");
    MLOG_WARN("warn");
    MLOG_ERROR("error");
    MLOG_RECORD_TIMESTAMP(std::to_string(count), "mlog_test", now());
    MLOG_RECORD_RELATION("result", std::to_string(count));
    count++;
  }
}
#else
int main(int argc, char *argv[]) {
  printf("Not deffine MLOG_ENABLE_ROS.\n");
  abort();
}
#endif
