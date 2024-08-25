#pragma once

#include "mlog/mlog.h"

namespace mlog {
namespace {

constexpr const char *get_data_type_name(MLogDataType type) {
  return type == MLogDataType::MLOG_DATA_TYPE_NORMAL
             ? "NORMAL"
             : type == MLogDataType::MLOG_DATA_TYPE_MSG_TIMESTAMP
                   ? "MSG_TIMESTAMP"
                   : type == MLogDataType::MLOG_DATA_TYPE_MSG_RELATION
                         ? "MSG_RELATION"
                         : "UNKNOWN";
}

} // namespace
} // namespace mlog
