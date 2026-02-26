# File: CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(LidarWorldModel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# -----------------------------
# Core library
# -----------------------------
add_library(wm_core STATIC
  src/core/util/config_loader.cpp
  src/core/util/repro_hash.cpp
  src/core/events/jsonl_event_sink.cpp
  src/core/model/node_runner.cpp
)

target_include_directories(wm_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(wm_core PUBLIC
  yaml-cpp
  Threads::Threads
)

# -----------------------------
# Adapters
# -----------------------------
add_library(wm_adapter_replay STATIC
  src/adapters/replay/replay_reader.cpp
)
target_include_directories(wm_adapter_replay PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(wm_adapter_replay PUBLIC wm_core)

add_library(wm_adapter_synth STATIC
  src/adapters/synth/synth_frame_source.cpp
)
target_include_directories(wm_adapter_synth PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(wm_adapter_synth PUBLIC wm_core)

add_library(wm_adapter_frame_dir STATIC
  src/adapters/frame_dir/frame_dir_source.cpp
)
target_include_directories(wm_adapter_frame_dir PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(wm_adapter_frame_dir PUBLIC wm_core)

# -----------------------------
# Executables
# -----------------------------
add_executable(wm_node
  src/apps/wm_node/main.cpp
)

target_include_directories(wm_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(wm_node PRIVATE
  wm_core
  wm_adapter_synth
  wm_adapter_frame_dir
)
