/***
 *Author:       Xiaodong
 *Version:      v3.0
 *Last Modified:2019-01-30
 *Note: only for Linux
 *Log:
 *      1.The header has some useful function
 */
#ifndef CUTE_H
#define CUTE_H

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#ifndef DEG2RAD
#define DEG2RAD(deg) (deg * 0.017453292519943295)
#endif

#ifndef RAD2DEG
#define RAD2DEG(rad) (rad * 57.29577951308232087721)
#endif

// return unix time in microseconds
inline long get_timeflag() {
  timeval tv;
  gettimeofday(&tv, NULL);
  return (long)(tv.tv_sec * 1000000 + tv.tv_usec);
}

// usage: declare outside the loop, and use .update(char *name) where you want
// to count FPS in a loop. Then "name:xxx fps" will display on terminal every
// second.
class FPS {
 private:
  unsigned int m_fps_;
  long m_timeStart_;
  void m_initial() {
    m_fps_ = 0;
    m_timeStart_ = get_timeflag();
  }

 public:
  FPS() { m_initial(); }

  void update(std::string name) {
    m_fps_++;
    if (get_timeflag() - m_timeStart_ >= 1e6) {
      printf("%s:%d fps\n", name.c_str(), m_fps_);
      m_initial();
    }
  }
};

#include <dirent.h>
#include <vector>
/*
 * @function: 获取dir目录下的所有文件名
 * @param: file_dir - 目录
 * @param: format - 获取文件后缀
 * @param: is_abspath - 输出绝对路径 或 仅文件名
 * @result：vector<string>类型
 */
inline std::vector<std::string> get_filenames(std::string file_dir,
                                              std::string format = "",
                                              bool is_abspath = false) {
  // file_dir += "/";
  std::vector<std::string> file_names;
  char abs_path[PATH_MAX];
  if (!realpath(file_dir.c_str(), abs_path)) {
    printf("the file '%s' is not exist\n", file_dir.c_str());
    return file_names;
  }
  std::string root_path = std::string(abs_path) + "/";

  DIR *dir;
  if ((dir = opendir(abs_path)) == NULL) {
    printf("Open dir error...%s\n", abs_path);
    return file_names;
  }

  struct dirent *ptr;
  while ((ptr = readdir(dir)) != NULL) {
    // printf("d_name:%s\n",ptr->d_name);
    if (strcmp(ptr->d_name, ".") == 0 ||
        strcmp(ptr->d_name, "..") == 0)  /// current dir OR parent dir
      continue;
    else if (ptr->d_type == 8)  /// file
    {
      std::string file_name =
          is_abspath ? root_path + ptr->d_name : ptr->d_name;
      if (format.empty())
        file_names.push_back(file_name);
      else if (file_name.substr(file_name.find_first_of('.') + 1) == format)
        file_names.push_back(file_name);
    } else if (ptr->d_type == 10)  /// link file
      continue;
    else if (ptr->d_type == 4)  /// dir
      continue;
  }
  closedir(dir);

  //排序，按从小到大排序
  sort(file_names.begin(), file_names.end());
  printf("find %d %s files\n", (int)file_names.size(), format.c_str());
  return file_names;
}

/*
 * @function: 获取dir目录下的所有文件夹的名称
 * @param: file_dir - 目录
 * @result：vector<string>类型
 */
inline std::vector<std::string> get_folder_name(std::string path) {
  path += "/";

  std::vector<std::string> dir_files;
  DIR *dir;
  if ((dir = opendir(path.c_str())) == NULL) {
    printf("Open dir error...%s\n", path.c_str());
    return dir_files;
  }

  struct dirent *ptr;

  while ((ptr = readdir(dir)) != NULL) {
    if (strcmp(ptr->d_name, ".") == 0 ||
        strcmp(ptr->d_name, "..") == 0)  /// current dir OR parent dir
      continue;
    if (ptr->d_type == 4)  /// dir
      dir_files.push_back(ptr->d_name);
  }
  closedir(dir);

  //排序，按从小到大排序
  sort(dir_files.begin(), dir_files.end());
  return dir_files;
}

inline std::string get_ascii_date()
{
    char* fileName = (char*)calloc(128, sizeof(char*));

    time_t timeNow;
    time(&timeNow);
    struct tm *pTime = localtime(&timeNow);

    sprintf(fileName, "%.4d-%.2d-%.2d-%.2d-%.2d-%.2d" ,
            1900 + pTime->tm_year,
            pTime->tm_mon + 1,
            pTime->tm_mday,
            pTime->tm_hour,
            pTime->tm_min,
            pTime->tm_sec);

    return std::string(fileName);
}

enum COLOR { RED, GREEN };

inline void print_color(const std::string &msg, COLOR color) {
  switch (color) {
    case RED:
      printf("\e[31m\e[1m %s \e[0m\n", msg.c_str());
      break;
    case GREEN:
      break;
  }
}

// string format:https://github.com/mmc1993/sformat
#include <algorithm>
#include <string>
#include <tuple>

template <class T>
inline void ToString(std::string &ret, T &&val) {
  ret.append(std::to_string(std::forward<T>(val)));
}

inline void ToString(std::string &ret, const std::string &val) {
  ret.append(val);
}

inline void ToString(std::string &ret, const char *val) { ret.append(val); }

template <int N>
struct SFormatN {
  static std::string Format(const char *fmt) { assert(false); }
};

template <>
struct SFormatN<0> {
  template <class... ARGS>
  static std::string Format(const char *fmt, const std::tuple<ARGS...> &) {
    return fmt;
  }
};

template <class... ARGS>
std::string SFormat(const char *fmt, const ARGS &... args) {
  const auto tuple = std::forward_as_tuple(args...);
  return SFormatN<sizeof...(args)>::Format(fmt, tuple);
}

#define FMT_N(idx)                      \
  case idx:                             \
    ToString(ret, std::get<idx>(args)); \
    break;

#define FMT_PARSE(N, ...)                                        \
  template <>                                                    \
  struct SFormatN<N> {                                           \
    template <class... ARGS>                                     \
    static std::string Format(const char *fmt,                   \
                              const std::tuple<ARGS...> &args) { \
      std::string ret;                                           \
      while (*fmt != '\0') {                                     \
        auto idx = -1;                                           \
        if (*fmt == '{') {                                       \
          idx = 0;                                               \
          ++fmt;                                                 \
          while (*fmt >= '0' && *fmt <= '9') {                   \
            idx *= 10;                                           \
            idx += (int)(*fmt++ - '0');                          \
          }                                                      \
          if (*fmt != '}')                                       \
            idx = -1;                                            \
          else                                                   \
            ++fmt;                                               \
        }                                                        \
        switch (idx) {                                           \
          __VA_ARGS__ default : ret.append(1, *fmt++);           \
          break;                                                 \
        }                                                        \
      }                                                          \
      return ret;                                                \
    }                                                            \
  };

FMT_PARSE(1, FMT_N(0))
FMT_PARSE(2, FMT_N(0) FMT_N(1))
FMT_PARSE(3, FMT_N(0) FMT_N(1) FMT_N(2))
FMT_PARSE(4, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3))
FMT_PARSE(5, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4))
FMT_PARSE(6, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5))
FMT_PARSE(7, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6))
FMT_PARSE(8, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                 FMT_N(7))
FMT_PARSE(9, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                 FMT_N(7) FMT_N(8))
FMT_PARSE(10, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                  FMT_N(7) FMT_N(8) FMT_N(9))
FMT_PARSE(11, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                  FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10))
FMT_PARSE(12, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                  FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11))
FMT_PARSE(13, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                  FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11) FMT_N(12))
FMT_PARSE(14, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                  FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11) FMT_N(12)
                      FMT_N(13))
FMT_PARSE(15, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                  FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11) FMT_N(12)
                      FMT_N(13) FMT_N(14))
FMT_PARSE(16, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                  FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11) FMT_N(12)
                      FMT_N(13) FMT_N(14) FMT_N(15))
FMT_PARSE(17, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                  FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11) FMT_N(12)
                      FMT_N(13) FMT_N(14) FMT_N(15) FMT_N(16))
FMT_PARSE(18, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                  FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11) FMT_N(12)
                      FMT_N(13) FMT_N(14) FMT_N(15) FMT_N(16) FMT_N(17))
FMT_PARSE(19,
          FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
              FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11) FMT_N(12) FMT_N(13)
                  FMT_N(14) FMT_N(15) FMT_N(16) FMT_N(17) FMT_N(18))
FMT_PARSE(20,
          FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
              FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11) FMT_N(12) FMT_N(13)
                  FMT_N(14) FMT_N(15) FMT_N(16) FMT_N(17) FMT_N(18) FMT_N(19))
FMT_PARSE(21, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                  FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11) FMT_N(12)
                      FMT_N(13) FMT_N(14) FMT_N(15) FMT_N(16) FMT_N(17)
                          FMT_N(18) FMT_N(19) FMT_N(20))
FMT_PARSE(22, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                  FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11) FMT_N(12)
                      FMT_N(13) FMT_N(14) FMT_N(15) FMT_N(16) FMT_N(17)
                          FMT_N(18) FMT_N(19) FMT_N(20) FMT_N(21))
FMT_PARSE(23, FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
                  FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11) FMT_N(12)
                      FMT_N(13) FMT_N(14) FMT_N(15) FMT_N(16) FMT_N(17)
                          FMT_N(18) FMT_N(19) FMT_N(20) FMT_N(21) FMT_N(22))
FMT_PARSE(24,
          FMT_N(0) FMT_N(1) FMT_N(2) FMT_N(3) FMT_N(4) FMT_N(5) FMT_N(6)
              FMT_N(7) FMT_N(8) FMT_N(9) FMT_N(10) FMT_N(11) FMT_N(12) FMT_N(13)
                  FMT_N(14) FMT_N(15) FMT_N(16) FMT_N(17) FMT_N(18) FMT_N(19)
                      FMT_N(20) FMT_N(21) FMT_N(22) FMT_N(23))

#endif  // CUTE_H
