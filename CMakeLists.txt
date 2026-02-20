# CMakeLists.txt
cmake_minimum_required(VERSION 3.22)

project(world_model
  VERSION 0.1.0
  DESCRIPTION "LiDAR world model"
  LANGUAGES CXX
)

# Keep WSL builds pure Linux. Do not discover Windows toolchains/libs via /mnt/c.
if(UNIX AND EXISTS "/mnt/c")
  set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH OFF)
  list(APPEND CMAKE_IGNORE_PREFIX_PATH "/mnt/c")
endif()

# -----------------------------
# Options
# -----------------------------
option(WM_BUILD_TESTS "Build tests" ON)
option(WM_ENABLE_WARNINGS "Enable compiler warnings" ON)
option(WM_ENABLE_WERROR "Treat warnings as errors" OFF)
option(WM_ENABLE_ASAN "Enable AddressSanitizer (debug/dev only)" OFF)
option(WM_ENABLE_UBSAN "Enable UndefinedBehaviourSanitizer (debug/dev only)" OFF)

# -----------------------------
# Global settings
# -----------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Predictable artefact locations
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# -----------------------------
# Helper functions
# -----------------------------
function(wm_enable_warnings target)
  if(NOT WM_ENABLE_WARNINGS)
    return()
  endif()

  if(MSVC)
    target_compile_options(${target} PRIVATE /W4)
    if(WM_ENABLE_WERROR)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target} PRIVATE
      -Wall -Wextra -Wpedantic
      -Wshadow -Wconversion -Wsign-conversion
      -Wnon-virtual-dtor -Wold-style-cast
      -Woverloaded-virtual -Wnull-dereference
    )
    if(WM_ENABLE_WERROR)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
endfunction()

function(wm_enable_sanitisers target)
  if(MSVC)
    return()
  endif()

  if(WM_ENABLE_ASAN)
    target_compile_options(${target} PRIVATE -fsanitize=address -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=address)
  endif()

  if(WM_ENABLE_UBSAN)
    target_compile_options(${target} PRIVATE -fsanitize=undefined -fno-omit-frame-pointer)
    target_link_options(${target} PRIVATE -fsanitize=undefined)
  endif()
endfunction()

# Scaffolding-friendly: only add sources if they exist.
function(wm_add_sources_if_exist target)
  foreach(src IN LISTS ARGN)
    if(EXISTS "${src}")
      target_sources(${target} PRIVATE "${src}")
    else()
      message(STATUS "WM: skipping missing source (expected during scaffolding): ${src}")
    endif()
  endforeach()
endfunction()

function(wm_add_executable_if_main_exists exe main_src)
  if(EXISTS "${main_src}")
    add_executable(${exe} "${main_src}")
    target_compile_features(${exe} PRIVATE cxx_std_20)
    target_include_directories(${exe} PRIVATE ${PROJECT_SOURCE_DIR}/include)
    wm_enable_warnings(${exe})
    wm_enable_sanitisers(${exe})
    set_target_properties(${exe} PROPERTIES OUTPUT_NAME ${exe})
  else()
    message(STATUS "WM: not creating target '${exe}' yet; missing ${main_src}")
  endif()
endfunction()

# -----------------------------
# Dependencies
# -----------------------------
find_package(yaml-cpp REQUIRED)

# -----------------------------
# Libraries
# -----------------------------
add_library(wm_core STATIC)
target_compile_features(wm_core PUBLIC cxx_std_20)
target_include_directories(wm_core
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(wm_core
  PUBLIC
    yaml-cpp
)

wm_enable_warnings(wm_core)
wm_enable_sanitisers(wm_core)

wm_add_sources_if_exist(wm_core
  ${PROJECT_SOURCE_DIR}/src/core/util/config_loader.cpp
  ${PROJECT_SOURCE_DIR}/src/core/util/repro_hash.cpp
  ${PROJECT_SOURCE_DIR}/src/core/events/jsonl_event_sink.cpp
  ${PROJECT_SOURCE_DIR}/src/core/model/node_runner.cpp
)

add_library(wm_adapter_replay STATIC)
target_compile_features(wm_adapter_replay PUBLIC cxx_std_20)
target_include_directories(wm_adapter_replay
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(wm_adapter_replay
  PUBLIC
    wm_core
)

wm_enable_warnings(wm_adapter_replay)
wm_enable_sanitisers(wm_adapter_replay)

wm_add_sources_if_exist(wm_adapter_replay
  ${PROJECT_SOURCE_DIR}/src/adapters/replay/replay_reader.cpp
)

# -----------------------------
# Executables
# -----------------------------
wm_add_executable_if_main_exists(
  wm_node
  ${PROJECT_SOURCE_DIR}/src/apps/wm_node/main.cpp
)


if(TARGET wm_node)
  target_link_libraries(wm_node PRIVATE wm_adapter_replay wm_core)
endif()

# -----------------------------
# Tests
# -----------------------------
if(WM_BUILD_TESTS)
  enable_testing()
  # Add Catch2/GoogleTest later; keep scaffold minimal for now.
endif()
