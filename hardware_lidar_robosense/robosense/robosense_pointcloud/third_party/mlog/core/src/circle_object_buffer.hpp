#pragma once
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>

namespace mlog {

namespace {
template <typename T> class CircleObjectBuffer {
public:
  enum class Status {
    OK = 0,
    TIMEOUT = 1,
    EMPTY = 2,
  };

public:
  CircleObjectBuffer(const size_t &max_size) : max_size_(max_size) {}

  bool is_empty() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return is_empty_without_lock();
  }

  size_t size() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    return size_without_lock();
  }

  Status push(const T &element) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    push_without_lock(element);
    data_cond_.notify_one();
    return Status::OK;
  }

  Status pop(T &element) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    data_cond_.wait([this] { return is_empty_without_lock() == false; });
    auto status = pop_without_lock(element);
    if (status != Status::OK) {
      return status;
    }
    return Status::OK;
  }

  Status pop_with_timeout(T &element, int timeout) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    if (data_cond_.wait_for(lock, std::chrono::milliseconds(timeout), [this] {
          return is_empty_without_lock() == false;
        })) {
      auto status = pop_without_lock(element);
      if (status != Status::OK) {
        return status;
      }
    } else {
      return Status::TIMEOUT;
    }
    return Status::OK;
  }

  Status pop_without_block(T &element) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    if (is_empty_without_lock()) {
      return Status::EMPTY;
    }
    auto status = pop_without_lock(element);
    REMOTE_CHECK_STATUS_RETURN(status);
    return Status::OK;
  }

private:
  bool is_empty_without_lock() { return data_.empty(); }

  size_t size_without_lock() { return data_.size(); }

  Status push_without_lock(const T &element) {
    data_.push_back(element);
    if (data_.size() > max_size_) {
      data_.pop_front();
    }
    return Status::OK;
  }

  Status pop_without_lock(T &element) {
    element = data_.front();
    data_.pop_front();
    return Status::OK;
  }

private:
  std::mutex data_mutex_;
  std::condition_variable data_cond_;
  std::deque<T> data_;
  size_t max_size_;
};

template <typename T>
using CircleObjectBufferPtr = std::shared_ptr<CircleObjectBuffer<T>>;
} // namespace
} // namespace mlog
