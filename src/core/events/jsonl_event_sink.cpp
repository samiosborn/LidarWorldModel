// File: src/core/events/jsonl_event_sink.cpp
#include "wm/core/events/jsonl_event_sink.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>

namespace wm {
namespace {

double ns_to_s(std::int64_t ns) { return static_cast<double>(ns) * 1e-9; }

std::string join_path(const std::string& a, const std::string& b) {
  namespace fs = std::filesystem;
  return (fs::path(a) / fs::path(b)).string();
}

// Your TimestampNs is a struct with a `.ns` field (no implicit cast).
std::int64_t as_i64(TimestampNs t) { return static_cast<std::int64_t>(t.ns); }

}  // namespace

JsonlEventSink::~JsonlEventSink() { close(); }

Status JsonlEventSink::open(const RunInfo& run) {
  close();

  std::error_code ec;
  std::filesystem::create_directories(run.out_dir, ec);
  if (ec) {
    return Status::io_error("failed creating out_dir '" + run.out_dir + "': " + ec.message());
  }

  const std::int64_t wall0 = as_i64(run.wall_start_time_ns);
  const std::int64_t t0 = as_i64(run.start_time_ns);

  path_ = join_path(run.out_dir, "events_" + std::to_string(wall0) + ".jsonl");
  latest_path_ = join_path(run.out_dir, "events_latest.jsonl");

  f_.open(path_, std::ios::out | std::ios::trunc);
  if (!f_.is_open()) return Status::io_error("failed opening '" + path_ + "'");

  latest_.open(latest_path_, std::ios::out | std::ios::trunc);
  if (!latest_.is_open()) return Status::io_error("failed opening '" + latest_path_ + "'");

  open_ = true;

  // Run header line (written to BOTH files).
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6);
  ss << "{"
     << "\"type\":\"run_started\","
     << "\"t_ns\":" << t0 << ","
     << "\"t_s\":" << ns_to_s(t0) << ","
     << "\"t_wall_ns\":" << wall0 << ","
     << "\"t_wall_s\":" << ns_to_s(wall0) << ","
     << "\"node_id\":\"" << run.node_id << "\","
     << "\"config_path\":\"" << run.config_path << "\","
     << "\"config_hash\":\"" << run.config_hash << "\","
     << "\"calibration_hash\":\"" << run.calibration_hash << "\""
     << "}";

  const Status w = write_line_(ss.str());
  if (!w.ok()) return w;
  (void)flush();

  return Status{};
}

Status JsonlEventSink::emit(const Event& e) {
  if (!open_) return Status::invalid_argument("JsonlEventSink::emit called while not open");

  const std::int64_t t = as_i64(e.t_ns);
  const std::int64_t tw = as_i64(e.t_wall_ns);

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6);

  ss << "{"
     << "\"type\":\"" << e.type << "\","
     << "\"t_ns\":" << t << ","
     << "\"t_s\":" << ns_to_s(t) << ","
     << "\"t_wall_ns\":" << tw << ","
     << "\"t_wall_s\":" << ns_to_s(tw);

  if (!e.message.empty()) {
    ss << ",\"message\":\"" << e.message << "\"";
  }

  ss << "}";

  return write_line_(ss.str());
}

Status JsonlEventSink::write_line_(const std::string& line) {
  f_ << line << "\n";
  latest_ << line << "\n";

  if (!f_.good()) return Status::io_error("failed writing to '" + path_ + "'");
  if (!latest_.good()) return Status::io_error("failed writing to '" + latest_path_ + "'");

  return Status{};
}

Status JsonlEventSink::flush() {
  if (!open_) return Status{};

  f_.flush();
  latest_.flush();

  if (!f_.good()) return Status::io_error("failed flushing '" + path_ + "'");
  if (!latest_.good()) return Status::io_error("failed flushing '" + latest_path_ + "'");

  return Status{};
}

void JsonlEventSink::close() {
  if (f_.is_open()) f_.close();
  if (latest_.is_open()) latest_.close();
  open_ = false;
}

}  // namespace wm