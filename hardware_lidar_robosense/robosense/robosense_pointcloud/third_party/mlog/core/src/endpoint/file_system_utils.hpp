#pragma once
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>

namespace {

inline std::string directory_name(const std::string &filename) {
  auto pos = filename.find_last_of("/");
  if (pos == std::string::npos) {
    return "./";
  } else {
    return filename.substr(0, pos);
  }
}

inline void make_directory(const std::string &dirname) {
  if (access(dirname.c_str(), W_OK) != 0) {
    auto result = mkdir(dirname.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (result != 0) {
      fprintf(stderr, "Create directory %s error.\n", dirname.c_str());
    }
  }
}

inline void make_recursive_directory(const std::string &dirname) {
  int pos = 0;
  while (true) {
    auto new_pos = dirname.find("/", pos);
    if (new_pos == std::string::npos) {
      break;
    }
    auto current_directory = dirname.substr(0, new_pos);
    if (access(dirname.c_str(), R_OK) != 0) {
      make_directory(current_directory);
    }
    pos = new_pos + 1;
  }
  make_directory(dirname);
}

inline bool get_file_info(const std::string &filename,
                          struct stat &logfeature) {
  if (lstat(filename.c_str(), &logfeature) == 0) {
    return true;
  } else {
    return false;
  }
}

inline void delete_file(const std::string &filename) {
  int err = remove(filename.c_str());
  if (err < 0) {
    fprintf(stderr, "[mlog] delete file %s is error:%d\n", filename.c_str(),
            errno);
  }
}
} // namespace
