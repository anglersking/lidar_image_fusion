project(perception_sample)
set(CMAKE_CXX_STANDARD 11)

file(GLOB CPP_FILES src/*.cpp)

foreach(SAMPLE_SOURCE ${CPP_FILES})
  string(REGEX REPLACE ".+/(.+)\\..*" "\\1" SAMPLE_NAME ${SAMPLE_SOURCE})
  add_executable(${SAMPLE_NAME} ${SAMPLE_SOURCE})
  target_link_libraries(${SAMPLE_NAME} mlog pthread)
endforeach(SAMPLE_SOURCE ${CPP_FILES})

