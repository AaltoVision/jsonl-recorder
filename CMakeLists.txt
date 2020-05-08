cmake_minimum_required(VERSION 3.0)
project(recorder)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wextra -O2")

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

add_library(recorder
  recorder.cpp
  )
target_link_libraries(recorder PRIVATE nlohmann_json::nlohmann_json)
