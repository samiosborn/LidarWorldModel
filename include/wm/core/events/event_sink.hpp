// File: include/wm/core/events/event_sink.hpp
#pragma once

#include <string>

#include "wm/core/status.hpp"
#include "wm/core/types.hpp"  // TimestampNs

namespace wm {

// Run metadata.
// Time contract:
//  - start_time_ns      : logical run time origin (relative, usually 0)
//  - wall_start_time_ns : wall-clock epoch time at run start (absolute)
struct RunInfo {
  std::string node_id;
  std::string config_path;
  std::string out_dir;

  std::string config_hash;
  std::string calibration_hash;

  TimestampNs start_time_ns{0};
  TimestampNs wall_start_time_ns{0};
};

// Event payload for MVP.
// Time contract:
//  - t_ns      : logical time since run start (relative, starts at 0)
//  - t_wall_ns : wall-clock epoch ns (absolute)
struct Event {
  std::string type;

  TimestampNs t_ns{0};
  TimestampNs t_wall_ns{0};

  std::string message;
};

class EventSink {
 public:
  virtual ~EventSink() = default;

  virtual Status open(const RunInfo& run) = 0;
  virtual Status emit(const Event& e) = 0;
  virtual Status flush() = 0;
  virtual void close() = 0;
};

}  // namespace wm