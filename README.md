# jsonl-recorder

Can be used to record sensor data and other data such as VIO frame metadata or GPS locations to a JSONL file.

Basic usage with CMake
```cmake
# path to a submodule pointing to this repository, or just a copy of these files
add_subdirectory("jsonl-recorder")
target_link_libraries(YOUR_TARGET recorder)
```

Dependencies: https://github.com/nlohmann/json
