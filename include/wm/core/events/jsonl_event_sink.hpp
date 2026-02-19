// File: include/wm/core/events/jsonl_event_sink.hpp
#pragma once

#include <fstream>
#include <string>

#include "wm/core/events/event_sink.hpp"
#include "wm/core/status.hpp"

namespace wm {

// JSON Lines event sink:
// - writes one JSON object per line
// - append-only
// - suitable for logs, regression outputs, and downstream ingestion
class JsonlEventSink final : public EventSink {
 public:
  JsonlEventSink() = default;
  ~JsonlEventSink() override { close(); }

  Status open(const RunInfo& run) override;
  Status emit(const Event& e) override;
  Status flush() override;
  void close() override;

  [[nodiscard]] const std::string& path() const noexcept { return path_; }

 private:
  std::string path_;
  std::ofstream ofs_;

  static std::string escape_json(const std::string& s);
};

}  // namespace wm
