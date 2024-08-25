#pragma once

#include "mlog/mlog.h"
#include <cstdlib>
#include <string>
#include <vector>

#ifdef MLOG_ENABLE_MFR
#include "mfr/mfr.h"
#include "mlog_mfrmsgs.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <mutex>
#include <regex>
#include <thread>
#include <unistd.h>

namespace {
static constexpr auto node_yaml = R"(
node_config:
  memory:
    total_virtual_memory_size_MB: 0
    total_shared_memory_size_MB: 0
)";

static constexpr auto machine_yaml = R"(
log:
  level: info
  file:
    file_name: ''
    file_type: write
    max_file_size: 0
    max_file_num: 1
  enable_stderr: true
  export_frequence: 0
)";

static constexpr auto mfr_master_uri = "mfrrpc://localhost:11300";

std::string topic;
int waiting_time_sec;
std::vector<std::string> contents;
mmessage::mlog_mfrmsgs::MFRMessageEntry data{};
mfr::MFRNodeHandle *node_handle;

uint64_t get_current_time_ns() {
  auto now = std::chrono::high_resolution_clock::now();
  uint64_t during =
      std::chrono::duration<uint64_t, std::nano>(now.time_since_epoch())
          .count();
  return during;
}

class LogNode : public mfr::MFRNode {
public:
  void on_init(mfr::MFRNodeHandle *handle) override {
    handle->trigger_manager().set_time_trigger(100);
    { // get publisher
      mfr::MFRPublisherConfig pub_config{};
      pub_config.topic_name = topic.c_str();
      pub_config.queue_size = 10;
      publisher_ =
          handle->communication_manager()
              .advertise<mmessage::mlog_mfrmsgs::MFRMessageLogs>(pub_config);
    }
    puts("node on_init()");
  }
  void on_finish() override { puts("node on_finish()"); }
  void on_running(const mfr::MFRNodeRunningInfo &info) override {
    std::unique_lock<std::mutex> lock(data_mutex_);

    mmessage::mlog_mfrmsgs::MFRMessageLogs logs{};

    auto header = logs.mutable_header();
    header.set_stamp(get_current_time_ns());

    for (auto content : contents) {
      mmessage::mlog_mfrmsgs::MFRMessageEntry data{};
      data.set_data(content.c_str());
      logs.add_entries(data);
    }

    publisher_->publish(logs);
  }

private:
  mfr::MFRPublisher *publisher_;
  std::mutex data_mutex_;
  mmessage::mlog_mfrmsgs::MFRMessageEntry data{};
};

MFR_REGISTER_NODE(LogNode, "node_log");

void register_nodes() {
  mfr::MFRNodeConfig node_config{};
  node_config.node_type = "node_log";
  node_config.node_name = "mfrnode";
  node_config.node_param_yaml = node_yaml;
  mfr::MFRNodeMachine::instance().register_node(node_config);
}

void init_machine(const mmemory::MFString &machine_port) {
  mfr::MFRMachineConfig machine_config{};
  machine_config.machine_url = machine_port;
  machine_config.machine_name = "mfrmlog_machine";
  machine_config.machine_param_yaml = machine_yaml;
  mfr::MFRNodeMachine::instance().init(machine_config);
}

void run_machine() { mfr::MFRNodeMachine::instance().run(); }

void stop_machine() {
  mfr::MFRNodeMachine::instance().stop();
  mfr::MFRNodeMachine::instance().join();
  mfr::MFRNodeMachine::instance().reset();
}

inline void mfr_endpoint_callback(const std::vector<mlog::MLogEntry> &entries,
                                  const std::string &topic_name,
                                  double init_waiting_time_sec, bool only_init,
                                  void *handle = nullptr) {
  static mfr::MFRPublisher *publisher = nullptr;
#ifdef MACHINE_ALREADY_EXIST
  if (publisher == nullptr) {
    if (handle == nullptr) {
      throw std::runtime_error("[MLOG_MFR] node_handle can not be null\n");
    } else {
      node_handle = (mfr::MFRNodeHandle *)handle;
    }
    { // get publisher
      mfr::MFRPublisherConfig pub_config{};
      if (topic_name == "")
        pub_config.topic_name = "mlog";
      else
        pub_config.topic_name = topic_name.c_str();
      pub_config.queue_size = 10;
      publisher =
          node_handle->communication_manager()
              .advertise<mmessage::mlog_mfrmsgs::MFRMessageLogs>(pub_config);
      std::this_thread::sleep_for(
          std::chrono::duration<double, std::milli>(waiting_time_sec * 1000));
    }
  }
  if (only_init)
    return;

  mmessage::mlog_mfrmsgs::MFRMessageLogs logs{};

  auto header = logs.mutable_header();
  header.set_stamp(get_current_time_ns());

  for (const auto &entry : entries) {
    std::string content = "";
    content += mlog::get_data_type_name(entry.type);
    content += " ";
    content += entry.data.c_str();

    mmessage::mlog_mfrmsgs::MFRMessageEntry data{};
    data.set_data(content.c_str());
    logs.add_entries(data);
  }

  publisher->publish(logs);
#else
  topic = "";
  waiting_time_sec = 0;
  if (contents.size() != 0)
    contents.clear();

  if (topic_name == "")
    topic = "mlog";
  else
    topic = topic_name;

  for (const auto &entry : entries) {
    std::string content = "";
    content += mlog::get_data_type_name(entry.type);
    content += " ";
    content += entry.data.c_str();
    contents.push_back(content);
  }

  waiting_time_sec = init_waiting_time_sec;

  register_nodes();
  mmemory::MFString machine_url = "";
  if (const char *env = std::getenv("MFR_MASTER_URI")) {
    machine_url = env;
  } else {
    machine_url = mfr_master_uri;
  }
  init_machine(machine_url);
  run_machine();
  std::this_thread::sleep_for(
      std::chrono::duration<double, std::milli>(waiting_time_sec * 1000));
  stop_machine();
#endif
}
} // namespace

#else
namespace {
inline void mfr_endpoint_callback(const std::vector<mlog::MLogEntry> &entries,
                                  const std::string &topic_name,
                                  double init_waiting_time_sec, bool only_init,
                                  void *handle = nullptr) {
  fprintf(stderr, "MFR Endpoint is invalid\n");
  std::abort();
}
} // namespace
#endif
