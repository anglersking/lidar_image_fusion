#pragma once
#include "common.hpp"
#include "file_system_utils.hpp"
#include "mlog/mlog.h"

namespace mlog {
namespace {
inline void stderr_endpoint_callback(const std::vector<MLogEntry> &entries) {
  for (auto &entry : entries) {
    fprintf(stderr, "%s %s\n", get_data_type_name(entry.type),
            entry.data.c_str());
  }
}

} // namespace
} // namespace mlog
