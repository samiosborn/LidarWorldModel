// File: include/wm/core/model/node_runner.hpp
#pragma once

#include <string>

#include "wm/core/config.hpp"
#include "wm/core/events/event_sink.hpp"
#include "wm/core/status.hpp"
#include "wm/core/types.hpp"

namespace wm {

// NodeRunner owns the run lifecycle.
// Today: open sink, write run header, manage heartbeat.
// Tomorrow: connect replay/live input -> mapping -> change detection -> events.
class NodeRunner {
 public:
  NodeRunner(Config cfg, std::string config_path);

  // Start a run: open sink + emit run_started header.
  Status start(EventSink& sink);

  // Emit a heartbeat when useful (caller decides cadence for now).
  Status emit_heartbeat(EventSink& sink, const std::string& message);

  // Stop is intentionally boring: flush + close. Keep shutdown predictable.
  void stop(EventSink& sink);

  [[nodiscard]] const Config& config() const noexcept { return cfg_; }
  [[nodiscard]] const std::string& config_path() const noexcept { return config_path_; }
  [[nodiscard]] TimestampNs start_time() const noexcept { return start_time_; }

 private:
  Config cfg_;
  std::string config_path_;
  TimestampNs start_time_{0};
};

}  // namespace wm