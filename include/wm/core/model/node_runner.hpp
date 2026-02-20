// File: include/wm/core/model/node_runner.hpp
#pragma once

#include <chrono>
#include <cstddef>
#include <string>

#include "wm/core/config.hpp"
#include "wm/core/events/event_sink.hpp"
#include "wm/core/status.hpp"
#include "wm/core/types.hpp"  // TimestampNs

namespace wm {

// NodeRunner owns lifecycle.
// Time contract:
//  - t_ns      = relative since run start (starts at 0) using steady clock
//  - t_wall_ns = absolute epoch ns anchored at start (wall_start + t_ns)
class NodeRunner {
 public:
  NodeRunner(Config cfg, std::string config_path);

  Status start(EventSink& sink);

  Status emit_heartbeat(EventSink& sink, const std::string& message);
  Status emit_heartbeat_at(EventSink& sink, TimestampNs t_ns, const std::string& message);

  void stop(EventSink& sink);

 private:
  static TimestampNs wall_now_epoch_ns();
  TimestampNs since_start_ns() const;

  // Convert relative run time to absolute wall epoch time:
  //   t_wall = wall_start + t_rel
  TimestampNs wall_at_(TimestampNs t_ns) const;

  static void prune_out_dir(const std::string& out_dir, std::size_t keep_last);

  Config cfg_;
  std::string config_path_;

  std::chrono::steady_clock::time_point t0_steady_{};
  TimestampNs t0_wall_ns_{0};
  bool started_{false};
};

}  // namespace wm