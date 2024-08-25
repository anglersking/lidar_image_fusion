#pragma once
#include "circle_object_buffer.hpp"
#include "dumper_helper.hpp"
#include "mlog/mlog.h"
#include <functional>
#include <thread>

namespace mlog {

namespace {

class BackgroundDumper {
public:
  BackgroundDumper()
      : request_buffer_(BUFFER_SIZE), worker_thread_(nullptr), stop_(true) {
    start();
  }

  ~BackgroundDumper() { stop(); }

  CircleObjectBuffer<LogDumpRequest> &get_buffer() { return request_buffer_; }

  static std::shared_ptr<BackgroundDumper> instance() {
    static std::shared_ptr<BackgroundDumper> instance =
        std::make_shared<BackgroundDumper>();
    return instance;
  }

private:
  void start() {
    if (stop_ == false)
      return;
    stop_ = false;
    worker_thread_ =
        std::make_shared<std::thread>(std::bind(&BackgroundDumper::run, this));
  }

  void stop() {
    if (stop_ == true)
      return;
    stop_ = true;
    if (worker_thread_ != nullptr) {
      worker_thread_->join();
      worker_thread_ = nullptr;
    }
  }

  void run() {
    LogDumpRequest request{};
    while (stop_ == false || request_buffer_.is_empty() == false) {
      auto status =
          request_buffer_.pop_with_timeout(request, BUFFER_READ_TIMEOUT_MS);
      if (status == CircleObjectBuffer<LogDumpRequest>::Status::OK) {
        dump_log_data(request);
      }
    }
  }

private:
  static constexpr auto BUFFER_SIZE = 1000;
  static constexpr auto BUFFER_READ_TIMEOUT_MS = 30;
  CircleObjectBuffer<LogDumpRequest> request_buffer_;
  std::shared_ptr<std::thread> worker_thread_;
  bool stop_;
};

} // namespace
} // namespace mlog
