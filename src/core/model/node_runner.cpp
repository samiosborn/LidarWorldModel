// File: src/core/model/node_runner.cpp
#include "wm/core/model/node_runner.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "wm/core/util/repro_hash.hpp"

namespace wm {
namespace {

bool is_digits(const std::string& s) {
  if (s.empty()) return false;
  for (char c : s) {
    if (c < '0' || c > '9') return false;
  }
  return true;
}

std::int64_t parse_events_epoch_ns_from_name(const std::string& name) {
  const std::string prefix = "events_";
  const std::string suffix = ".jsonl";

  // Never touch the stable tail target.
  if (name == "events_latest.jsonl") return -1;

  if (name.rfind(prefix, 0) != 0) return -1;
  if (name.size() <= prefix.size() + suffix.size()) return -1;
  if (name.substr(name.size() - suffix.size()) != suffix) return -1;

  const std::string mid =
      name.substr(prefix.size(), name.size() - prefix.size() - suffix.size());
  if (!is_digits(mid)) return -1;

  try {
    return std::stoll(mid);
  } catch (...) {
    return -1;
  }
}

}  // namespace

NodeRunner::NodeRunner(Config cfg, std::string config_path)
    : cfg_(std::move(cfg)), config_path_(std::move(config_path)) {}

TimestampNs NodeRunner::wall_now_epoch_ns() {
  using clock = std::chrono::system_clock;
  const auto now = clock::now().time_since_epoch();
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  return TimestampNs{static_cast<std::int64_t>(ns)};
}

TimestampNs NodeRunner::since_start_ns() const {
  if (!started_) return TimestampNs{0};
  const auto now = std::chrono::steady_clock::now();
  const auto ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now - t0_steady_).count();
  return TimestampNs{static_cast<std::int64_t>(ns)};
}

void NodeRunner::prune_out_dir(const std::string& out_dir, std::size_t keep_last) {
  namespace fs = std::filesystem;

  std::error_code ec;
  if (!fs::exists(out_dir, ec)) return;

  struct Entry {
    std::int64_t key_epoch_ns;
    fs::path path;
  };

  std::vector<Entry> files;
  for (const auto& it : fs::directory_iterator(out_dir, ec)) {
    if (ec) return;
    if (!it.is_regular_file(ec)) continue;

    const std::string name = it.path().filename().string();
    const std::int64_t k = parse_events_epoch_ns_from_name(name);
    if (k < 0) continue;

    files.push_back(Entry{k, it.path()});
  }

  if (files.size() <= keep_last) return;

  // Newest first, delete the tail.
  std::sort(files.begin(), files.end(),
            [](const Entry& a, const Entry& b) { return a.key_epoch_ns > b.key_epoch_ns; });

  for (std::size_t i = keep_last; i < files.size(); ++i) {
    fs::remove(files[i].path, ec);
    ec.clear();  // best-effort housekeeping
  }
}

Status NodeRunner::start(EventSink& sink) {
  prune_out_dir(cfg_.output.out_dir, /*keep_last=*/50);

  t0_steady_ = std::chrono::steady_clock::now();
  t0_wall_ns_ = wall_now_epoch_ns();
  started_ = true;

  RunInfo run;
  run.node_id = cfg_.node_id;
  run.config_path = config_path_;
  run.out_dir = cfg_.output.out_dir;

  run.config_hash = compute_config_hash(cfg_);
  run.calibration_hash = compute_calibration_hash(cfg_.calibration);

  // Contract: logical time starts at zero. Wall time is absolute epoch.
  run.start_time_ns = TimestampNs{0};
  run.wall_start_time_ns = t0_wall_ns_;

  return sink.open(run);
}

Status NodeRunner::emit_heartbeat(EventSink& sink, const std::string& message) {
  return emit_heartbeat_at(sink, since_start_ns(), message);
}

Status NodeRunner::emit_heartbeat_at(EventSink& sink, TimestampNs t_ns, const std::string& message) {
  Event e;
  e.type = "heartbeat";
  e.t_ns = t_ns;
  e.t_wall_ns = wall_now_epoch_ns();
  e.message = message;
  return sink.emit(e);
}

Status NodeRunner::emit_event(EventSink& sink, const std::string& type, const std::string& message) {
  Event e;
  e.type = type;
  e.t_ns = since_start_ns();
  e.t_wall_ns = wall_now_epoch_ns();
  e.message = message;
  return sink.emit(e);
}

void NodeRunner::stop(EventSink& sink) {
  (void)sink.flush();
  sink.close();
  started_ = false;
}

}  // namespace wm