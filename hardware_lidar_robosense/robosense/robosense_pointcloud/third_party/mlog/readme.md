# wanshannnt Log Toolkit

## 1. 简介

  该工具是一款加强版日志记录工具。本工具通过日志记录的方式，更近一步地实现了一系列的系统分析功能，用于辅助分析系统状态。

  使用该工具可以实现的一些常见功能如下：
  
  1. 常规日志功能：可以按级别记录字符串日志，还可以控制写文件频率以提高性能。
  2. 代码耗时分析：可以记录代码段在不同线程、进程间的运行时间，以及他们调用的时间关系，供离线可视化分析。
  3. 延时分析功能：可以按照特定格式记录系统内部消息之间的 “关系” 和 “时间点” 信息，供离线分析消息传递延时。

## 2. 快速上手

  ``` bash
  mkdir build && cd build
  cmake .. && make 
  ./sample/cpp/log # 运行样例
  ```

## 3. 详细功能介绍 

### 3.1 常规日志功能

  该功能为常规的日志记录功能，可以控制日志等级，来输出日志。同时具有一系列的高级设置，如控制输出频率来提高日志输出性能，会先在内存中缓存一段时间再予以写入文件。

  使用方式通过 cpp 引用头文件库，在代码中主动使用 MLOG_INFO() 等日志输出指令打印日志：

  ```cpp
  mlog::MLogManager::get_settings().export_frequence = 10; // 把输出频率设置为 10hz
  MLOG_DEBUG("debug"); 
  MLOG_INFO("info");
  MLOG_WARN("warn");
  MLOG_ERROR("error");
  ```

  完整使用样例如下：
  ```bash
  mkdir build && cd build
  cmake .. && make 
  ./sample/cpp/log # 运行样例

  # 可以看到输出了两个文件 log1.txt 和 log2.txt 可以查看 sample 源代码来了解通过日志等级来控制输出的原理。
  ```

### 3.2 代码耗时分析

  该功能主要靠日志方式，记录下每段代码运行开始时的时间戳/结束时的时间戳。然后离线使用记录的信息，使用可视化工具分析出代码段的耗时/调用关系。

  使用方式通过 cpp 引用头文件库，在代码中主动使用 profiling 代码：

  ```cpp
  {
    MLOG_PROFILING("func"); // 运行到当前的时候，会记录下 func 开始时间戳。
    do_your_task();
  }  // 运行到作用域结束的时候，自动触发 MLOG_PROFILING 的析构函数，会记录下 func 结束时间戳。
  ```
 
  完整使用样例如下：
  ```bash
  mkdir build && build
  cmake .. -DMLOG_ENABLE_PROFILING=ON && make # 需要开启 MLOG_ENABLE_PROFILING 的开关编译，否则不会生成 PROFILING 代码。
  MLOG_PROFILING_PATH="./perf.json" ./sample/cpp/profiling # 需要通过环境变量 MLOG_PROFILING_PATH 指定时间记录输出的文件路径，否则将没有日志输出打印。

  # sample 会输出 ./perf.json.<timestamp>.<process_id> 的文件，该文件可以用 Chrome 浏览器的链接 chrome://tracing 工具进行可视化解析。 
  ```
  
  值的注意的是，耗时分析输出格式为标准的 Google Trace Event Format. 因此与内置的 mlog 日志格式有差异，所以通过环境变量单独控制输出。

  同时在正式 release 版本中，需要关闭 -DMLOG_ENABLE_PROFILING 开关进行编译，这样 MLOG_PROFILING 宏将解析为空，才不会影响软件性能。
  

### 3.3 延时分析功能

  该功能主要靠日志方式，记录下消息之间的“关系”和“时间点”。然后离线用这些记录的消息，组建出消息流图，从而分析从输入到输出，消息流时延是如何分布的。

  使用方式通过 cpp 引用头文件库，在代码中主动打印 “时间点” 和 “关系”，如下：

  ```cpp
  MLOG_RECORD_TIMESTAMP(msg_id, event, timestamp); // 记录事件时间戳
  MLOG_RECORD_RELATION(current_msg_id, target_msg_id); // 记录消息间关联关系
  ```

  接着使用分析工具，对输出的日志文件离线进行进行解析，复原层消息流网络图，完整使用方式如下：
  
  ``` bash
  mkdir build && cd build
  cmake .. && make 
  ./sample/cpp/delay_analysis # sample 是一个多线程例子，使用到了 MLog 输出关键信息。
  ./sample/cpp/delay_analysis 2> output.txt # 将输出信息导入到文件中
  ../sample/python/sample_from_file.py -i output.txt # 使用分析工具分析出延时
  ```
  
  ROS 环境使用样例如下：
  
  1. 在代码中的启用 ros 的 endpoint
  ```
  mlog::MLog_set_endpoint(mlog::MLogEndpoint::MLOG_ENDPOINT_ROS_TOPIC); 
  ```

  2. 编译时打开 -DMLOG_ENABLE_ROS=ON
  ```
  catkin_make -DMLOG_ENABLE_ROS=ON # 或在项目 CMakeLists.txt 中启用 set(MLOG_ENABLE_ROS ON)
  ```

  MFR 环境使用样例如下：

  
  MFR 环境涉及如下几个宏：
  |宏|介绍
  |---|:---
  |*MLOG_ENABLE_MFR*|（必填）表示开启 MLOG 的 MFR 环境。
  |*MFRUNTIME_PATH*|（必填）需要依赖的 MFR 环境的绝对路径。
  |*MFRUNTIME_BUILD_PATH*|（必填）需要依赖的 `mfruntime.so` 的绝对路径
  |*MACHINE_ALREADY_EXIST*|（选填）在 MLOG 单模块验证时，不需要填写。在嵌入算法模块使用时，需要设置 `MACHINE_ALREADY_EXIST=ON` ，表示已存在启动的 machine 。
    
  1. 拉取 mfr-release
  2. 编译时打开 -DMLOG_ENABLE_MFR=ON 并且需要指定 `MFRUNTIME_PATH` 和 `MFRUNTIME_BUILD_PATH` 两个绝对路径。
  `MFRUNTIME_PATH` 指定所依赖的 MFR 环境的绝对路径，`MFRUNTIME_BUILD_PATH` 指定所依赖的 mfruntime.so 的绝对路径。

      示例如下：此时假设 `MFRUNTIME_PATH` 为 `~/mfr-release` , `MFRUNTIME_BUILD_PATH` 为 `~/mfr-release/lib/Linux-x86_64-gcc5.4` ,则输入命令
      ```
      cmake .. -DMLOG_ENABLE_MFR=ON -DMFRUNTIME_PATH="~/mfr-release/" -DMFRUNTIME_BUILD_PATH="~/mfr-release/lib/Linux-x86_64-gcc5.4"
      ```
3. 在 mlog 单模块验证时，需要用户手动执行 `export PYTHONPATH` 的操作，将 mlog 下的消息类型放入 PYHTONPATH 中。
      示例如下：此时假设 mlog 编译后得到的消息类型的python文件位于 `~/mlog/build/mfrproto/code/py` ，则输入命令
      ```
      export PYTHONPATH=${PYTHONPATH}:~/mlog/build/mfrproto/code/py
      ```
      将 mlog 的 message type 加入 `PYTHONPATH` 。

    在算法模块集成的环境中验证 mlog 时，需要用户执行 `source mfr_setup.bash`。并且在指定 `MFRUNTIME_PATH` 和 `MFRUNTIME_BUILD_PATH` 两个绝对路径的基础上，额外打开 `MACHINE_ALREADY_EXIST` 宏。
4. 运行 `./sample/cpp/log_mfr` ，即可在 MFR 环境下使用 mlog ，并可以与 mfrtopic 工具同步。