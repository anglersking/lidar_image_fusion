#include "mlog/mlog.h"
#include "log_manager_normal_impl.hpp"
#include <cassert>
#include <chrono>
#include <memory>
#include <mutex>

namespace mlog {
std::shared_ptr<MLogManager> MLogManager::make() {
  return std::make_shared<MLogManagerNormalImpl>(settings_);
}

std::shared_ptr<MLogManager> MLogManager::make(const MLogSettings &settings) {
  return std::make_shared<MLogManagerNormalImpl>(settings);
}

MLogSettings MLogManager::settings_{};
void MLogManager::set_settings(MLogSettings settings) { settings_ = settings; }

MLogSettings &MLogManager::get_settings() { return settings_; }

} // namespace mlog
