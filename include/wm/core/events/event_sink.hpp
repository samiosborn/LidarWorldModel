// File: include/wm/core/events/event_sink.hpp
#pragma once

#include <string>

#include "wm/core/status.hpp"
#include "wm/core/types.hpp"

namespace wm {

// Minimal event model for now.
// Keep output stable and boring; evolve by adding fields (not breaking existing ones).

struct RunInfo {
  NodeId node_id;
  std::string config_path;
  std::string out_dir;

  // Filled later when repro hashing lands.
  std::string config_hash;
  std::string calibration_hash;

  TimestampNs start_time;
};

struct Event {
  std::string type;     // e.g. "run_started", "heartbeat", "obstacle_added"
  TimestampNs timestamp;

  // Spatial fields (optional). If frame is empty, treat as non-spatial event.
  FrameName frame;
  AABB aabb;

  std::string message;  // optional human-readable hint
  double confidence = 0.0;
  double persistence_s = 0.0;
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
