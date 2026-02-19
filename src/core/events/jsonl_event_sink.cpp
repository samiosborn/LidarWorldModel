// File: src/core/events/jsonl_event_sink.cpp
#include "wm/core/events/jsonl_event_sink.hpp"

#include <filesystem>
#include <iomanip>
#include <sstream>

namespace wm {
namespace fs = std::filesystem;

static double ns_to_s(const TimestampNs& t) {
  return static_cast<double>(t.ns) / 1'000'000'000.0;
}

std::string JsonlEventSink::escape_json(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 8);

  for (const char c : s) {
    switch (c) {
      case '\"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        // Allow standard printable ASCII; escape control chars.
        if (static_cast<unsigned char>(c) < 0x20) {
          std::ostringstream oss;
          oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(static_cast<unsigned char>(c));
          out += oss.str();
        } else {
          out += c;
        }
    }
  }
  return out;
}

Status JsonlEventSink::open(const RunInfo& run) {
  close();

  try {
    fs::create_directories(run.out_dir);
  } catch (const std::exception& e) {
    return Status::io_error(std::string("failed to create out_dir: ") + e.what());
  }

  // One file per run to avoid mixing runs when iterating quickly.
  // Use start_time (epoch ns) for uniqueness and easy sorting.
  const std::string filename = "events_" + std::to_string(run.start_time.ns) + ".jsonl";
  path_ = (fs::path(run.out_dir) / filename).string();

  ofs_.open(path_, std::ios::out | std::ios::app);
  if (!ofs_.is_open()) {
    return Status::io_error("failed to open event log for append: " + path_);
  }

  // Write run header (as a normal event line).
  ofs_ << "{"
       << "\"type\":\"run_started\","
       << "\"t_ns\":" << run.start_time.ns << ","
       << "\"t_s\":" << std::fixed << std::setprecision(6) << ns_to_s(run.start_time) << ","
       << "\"node_id\":\"" << escape_json(run.node_id) << "\","
       << "\"config_path\":\"" << escape_json(run.config_path) << "\","
       << "\"config_hash\":\"" << escape_json(run.config_hash) << "\","
       << "\"calibration_hash\":\"" << escape_json(run.calibration_hash) << "\""
       << "}\n";

  ofs_.flush();
  return Status::ok_status();
}

Status JsonlEventSink::emit(const Event& e) {
  if (!ofs_.is_open()) {
    return Status::internal("JsonlEventSink::emit called before open()");
  }

  // Minimal JSON object. Only include spatial fields if frame is set.
  ofs_ << "{"
       << "\"type\":\"" << escape_json(e.type) << "\","
       << "\"t_ns\":" << e.timestamp.ns << ","
       << "\"t_s\":" << std::fixed << std::setprecision(6) << ns_to_s(e.timestamp);

  if (!e.frame.empty()) {
    ofs_ << ",\"frame\":\"" << escape_json(e.frame) << "\""
         << ",\"aabb\":{"
         << "\"min\":{" << "\"x\":" << e.aabb.min.x << ",\"y\":" << e.aabb.min.y << ",\"z\":" << e.aabb.min.z << "},"
         << "\"max\":{" << "\"x\":" << e.aabb.max.x << ",\"y\":" << e.aabb.max.y << ",\"z\":" << e.aabb.max.z << "}"
         << "}";
  }

  if (!e.message.empty()) {
    ofs_ << ",\"message\":\"" << escape_json(e.message) << "\"";
  }

  if (e.confidence > 0.0) {
    ofs_ << ",\"confidence\":" << e.confidence;
  }

  if (e.persistence_s > 0.0) {
    ofs_ << ",\"persistence_s\":" << e.persistence_s;
  }

  ofs_ << "}\n";
  return Status::ok_status();
}

Status JsonlEventSink::flush() {
  if (!ofs_.is_open()) return Status::ok_status();
  ofs_.flush();
  return Status::ok_status();
}

void JsonlEventSink::close() {
  if (ofs_.is_open()) {
    ofs_.flush();
    ofs_.close();
  }
  path_.clear();
}

}  // namespace wm
