cmake_minimum_required(VERSION 3.0)
set(LIBNAME jsonl-recorder)
project(${LIBNAME})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wextra -O2")

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(${LIBNAME} recorder.cpp)
set_target_properties(${LIBNAME} PROPERTIES PUBLIC_HEADER "recorder.hpp;types.hpp")
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(json ${CMAKE_CURRENT_BINARY_DIR}/nlohmann_json)
target_link_libraries(${LIBNAME} PRIVATE nlohmann_json::nlohmann_json)

install(TARGETS ${LIBNAME}
  ARCHIVE DESTINATION lib
  PUBLIC_HEADER DESTINATION include/${LIBNAME})
