// File: src/core/model/node_runner.cpp
#include "wm/core/model/node_runner.hpp"

#include <chrono>

#include "wm/core/util/repro_hash.hpp"

namespace wm {
namespace {

TimestampNs now_epoch_ns() {
  // Using system_clock keeps logs comparable across processes/machines.
  // Deterministic replay will later use dataset timestamps for event time.
  using clock = std::chrono::system_clock;
  const auto now = clock::now().time_since_epoch();
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  return TimestampNs{static_cast<std::int64_t>(ns)};
}

}  // namespace

NodeRunner::NodeRunner(Config cfg, std::string config_path)
    : cfg_(std::move(cfg)), config_path_(std::move(config_path)) {}

Status NodeRunner::start(EventSink& sink) {
  start_time_ = now_epoch_ns();

  RunInfo run;
  run.node_id = cfg_.node_id;
  run.config_path = config_path_;
  run.out_dir = cfg_.output.out_dir;

  // Fingerprints belong in every run header. When something breaks at 2am,
  // you want to know exactly what config/calibration produced the mess.
  run.config_hash = compute_config_hash(cfg_);
  run.calibration_hash = compute_calibration_hash(cfg_.calibration);

  run.start_time = start_time_;

  return sink.open(run);
}

Status NodeRunner::emit_heartbeat(EventSink& sink, const std::string& message) {
  Event e;
  e.type = "heartbeat";
  e.timestamp = now_epoch_ns();
  e.message = message;
  return sink.emit(e);
}

void NodeRunner::stop(EventSink& sink) {
  // Shutdown should be boring. If we can't flush, there's not much we can do.
  (void)sink.flush();
  sink.close();
}

}  // namespace wm