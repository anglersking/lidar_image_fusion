#include "mlog/profiling.h"
#include <cstdio>
#include <random>

void func2() {
  MLOG_PROFILING("func2"); // insert this line wherever you want to profile
  static std::default_random_engine e;
  std::uniform_int_distribution<int> u(50, 100);
  std::this_thread::sleep_for(std::chrono::milliseconds(u(e)));
}

void func1() {
  MLOG_PROFILING("func1"); // insert this line wherever you want to profile
  func2();
};

int main() {
  auto thread1 = std::thread([]() {
    const auto time = 90;
    for (int i = 0; i < time; ++i) {
      printf("thread1 %d/%d\n", i, time);
      func1();
    }
  });
  auto thread2 = std::thread([]() {
    const auto time = 100;
    for (int i = 0; i < time; ++i) {
      printf("thread2 %d/%d\n", i, time);
      func1();
    }
  });

  thread1.join();
  thread2.join();
}
