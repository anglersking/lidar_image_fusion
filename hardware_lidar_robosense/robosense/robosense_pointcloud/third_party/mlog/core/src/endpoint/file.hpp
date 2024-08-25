#pragma once
#include "common.hpp"
#include "file_system_utils.hpp"
#include "mlog/file_helper.h"
#include "mlog/mlog.h"

namespace mlog {
namespace {
inline void
file_endpoint_callback(const std::vector<MLogEntry> &entries,
                       const std::shared_ptr<MLogFileHelper> &file_helper) {
  file_helper->write_log(entries);
}

} // namespace
} // namespace mlog
