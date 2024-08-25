#ifndef COMMON_UTILS_H
#define COMMON_UTILS_H

#include <sys/time.h> // gettimeofday
#include <string>
#include <iostream>
using namespace std;

//获取字符串时间
inline string gettime() {
  struct timeval tv;  
  char buf[64];  
  gettimeofday(&tv, NULL);  
  strftime(buf, sizeof(buf)-1, "%Y-%m-%d-%H-%M-%S", localtime(&tv.tv_sec));  
  return buf;
}

#endif // COMMON_UTILS_H