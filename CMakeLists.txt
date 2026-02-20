# CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

project(LidarWorldModel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# -----------------------------
# Options
# -----------------------------
option(WM_WARNINGS "Enable common compiler warnings" ON)

# -----------------------------
# Dependencies
# -----------------------------
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

# -----------------------------
# Helper functions
# -----------------------------
function(wm_apply_warnings target_name)
  if (NOT WM_WARNINGS)
    return()
  endif()

  if (MSVC)
    target_compile_options(${target_name} PRIVATE /W4)
  else()
    target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

# -----------------------------
# Core library
# -----------------------------
add_library(wm_core STATIC
  src/core/util/config_loader.cpp
  src/core/util/repro_hash.cpp
  src/core/events/jsonl_event_sink.cpp
  src/core/model/node_runner.cpp
)

target_include_directories(wm_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(wm_core
  PUBLIC
    yaml-cpp
    Threads::Threads
)

target_compile_features(wm_core PUBLIC cxx_std_20)
wm_apply_warnings(wm_core)

# -----------------------------
# Adapters
# -----------------------------
add_library(wm_adapter_replay STATIC
  src/adapters/replay/replay_reader.cpp
)

target_include_directories(wm_adapter_replay
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

target_link_libraries(wm_adapter_replay
  PUBLIC
    wm_core
)

target_compile_features(wm_adapter_replay PUBLIC cxx_std_20)
wm_apply_warnings(wm_adapter_replay)

# -----------------------------
# Executables
# -----------------------------
add_executable(wm_node
  src/apps/wm_node/main.cpp
)

target_link_libraries(wm_node
  PRIVATE
    wm_core
    wm_adapter_replay
)

target_compile_features(wm_node PRIVATE cxx_std_20)
wm_apply_warnings(wm_node)