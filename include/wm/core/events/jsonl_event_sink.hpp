// File: include/wm/core/events/jsonl_event_sink.hpp
#pragma once

#include <fstream>
#include <string>

#include "wm/core/events/event_sink.hpp"
#include "wm/core/status.hpp"

namespace wm {

// JSONL sink for events.
// Writes every event line to:
//   1) a unique per-run file: events_<wall_start_time_ns>.jsonl
//   2) a stable "latest" file: events_latest.jsonl (truncated each run)
class JsonlEventSink final : public EventSink {
 public:
  JsonlEventSink() = default;
  ~JsonlEventSink() override;

  const std::string& path() const { return path_; }
  const std::string& latest_path() const { return latest_path_; }

  Status open(const RunInfo& run) override;
  Status emit(const Event& e) override;
  Status flush() override;
  void close() override;

 private:
  Status write_line_(const std::string& line);

  bool open_{false};

  std::string path_;
  std::string latest_path_;

  std::ofstream f_;
  std::ofstream latest_;
};

}  // namespace wm