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
  mlog::MLogManager::get_settings().export_frequence = 5;
  struct Data {
    std::mutex mutex;
    std::queue<std::string> queue;
  } pipe;

  std::thread producer([&pipe]() {
    for (int i = 0; i < 10; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(90));
      std::string msg_id = std::string("msg") + std::to_string(i);
      {
        std::unique_lock<std::mutex> lck(pipe.mutex);
        pipe.queue.push(msg_id);
      }
      MLOG_RECORD_TIMESTAMP(msg_id, "send", now());
    }
  });

  std::thread receiver([&pipe]() {
    std::string result_id = "result0";
    for (int i = 0; i < 10; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(90));
      std::string msg_id = std::string("msg") + std::to_string(i);
      {
        std::unique_lock<std::mutex> lck(pipe.mutex);
        pipe.queue.push(msg_id);
      }
      MLOG_RECORD_TIMESTAMP(msg_id, "recv", now());
      MLOG_RECORD_RELATION(result_id, msg_id);
    }
    MLOG_RECORD_TIMESTAMP(result_id, "output", now());
  });

  auto settings = mlog::MLogManager::get_settings();
  mlog::MLogSettings new_settings(settings);
  new_settings.endpoint = mlog::MLOG_ENDPOINT_FILE | mlog::MLOG_ENDPOINT_STDERR;
  new_settings.level = mlog::MLOG_LEVEL_ERROR;
  new_settings.log_file = "1.txt";
  mlog::MLogManager::set_settings(new_settings);
  MLOG_INFO("info");
  MLOG_DEBUG("debug");
  MLOG_WARN("warn");
  MLOG_ERROR("error");
  mlog::MLogManager::set_settings(settings);

  new_settings.log_file = "2.txt";
  new_settings.endpoint = mlog::MLOG_ENDPOINT_FILE;
  new_settings.level = mlog::MLOG_LEVEL_DEBUG;
  mlog::MLogManager::set_settings(new_settings);
  MLOG_DEBUG("2.debug");
  MLOG_INFO("2.info");
  mlog::MLogManager::set_settings(settings);

  producer.join();
  receiver.join();
}
