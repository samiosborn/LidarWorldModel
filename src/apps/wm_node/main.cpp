// src/apps/wm_node/main.cpp
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <chrono>

#include "wm/core/config.hpp"
#include "wm/core/config_loader.hpp"
#include "wm/core/status.hpp"
#include "wm/core/events/jsonl_event_sink.hpp"

namespace {

void print_usage(const char* argv0) {
  std::cout
      << "LiDAR World Model\n\n"
      << "Usage:\n"
      << "  " << argv0 << " --config <path> [options]\n\n"
      << "Options:\n"
      << "  --config <path>          Path to YAML config (required)\n"
      << "  --mode <replay|live>     Override mode\n"
      << "  --dataset <path>         Override replay.dataset_path\n"
      << "  --out_dir <path>         Override output.out_dir\n"
      << "  --node_id <id>           Override node_id\n"
      << "  --help                   Show this help\n\n"
      << "Examples:\n"
      << "  " << argv0 << " --config configs/profiles/desktop_dev.yaml\n"
      << "  " << argv0 << " --config configs/profiles/desktop_dev.yaml --dataset data/datasets/golden_runs/no_change/run_001\n";
}

double ns_to_seconds(std::int64_t ns) {
  return static_cast<double>(ns) / 1'000'000'000.0;
}

wm::TimestampNs now_epoch_ns() {
  using clock = std::chrono::system_clock;
  const auto now = clock::now().time_since_epoch();
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  return wm::TimestampNs{static_cast<std::int64_t>(ns)};
}


wm::Result<wm::RunMode> parse_mode(const std::string& s) {
  const auto lower = [&]() {
    std::string t = s;
    for (char& c : t) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return t;
  }();

  if (lower == "replay") return wm::Result<wm::RunMode>::ok(wm::RunMode::kReplay);
  if (lower == "live") return wm::Result<wm::RunMode>::ok(wm::RunMode::kLive);
  return wm::Result<wm::RunMode>::err(wm::Status::invalid_argument("unknown mode: " + s));
}

struct Args {
  std::string config_path;

  // Optional overrides
  std::string mode;
  std::string dataset_path;
  std::string out_dir;
  std::string node_id;
};

wm::Result<Args> parse_args(int argc, char** argv) {
  Args a;

  std::vector<std::string> v;
  v.reserve(static_cast<std::size_t>(argc));
  for (int i = 0; i < argc; ++i) v.emplace_back(argv[i]);

  for (int i = 1; i < argc; ++i) {
    const std::string& tok = v[static_cast<std::size_t>(i)];

    if (tok == "--help" || tok == "-h") {
      return wm::Result<Args>::err(wm::Status::invalid_argument("help"));
    }

    auto require_value = [&](const std::string& flag) -> wm::Result<std::string> {
      if (i + 1 >= argc) {
        return wm::Result<std::string>::err(
            wm::Status::invalid_argument("missing value for " + flag));
      }
      return wm::Result<std::string>::ok(v[static_cast<std::size_t>(++i)]);
    };

    if (tok == "--config") {
      auto r = require_value(tok);
      if (!r.ok()) return wm::Result<Args>::err(r.status());
      a.config_path = r.take_value();
      continue;
    }
    if (tok == "--mode") {
      auto r = require_value(tok);
      if (!r.ok()) return wm::Result<Args>::err(r.status());
      a.mode = r.take_value();
      continue;
    }
    if (tok == "--dataset") {
      auto r = require_value(tok);
      if (!r.ok()) return wm::Result<Args>::err(r.status());
      a.dataset_path = r.take_value();
      continue;
    }
    if (tok == "--out_dir") {
      auto r = require_value(tok);
      if (!r.ok()) return wm::Result<Args>::err(r.status());
      a.out_dir = r.take_value();
      continue;
    }
    if (tok == "--node_id") {
      auto r = require_value(tok);
      if (!r.ok()) return wm::Result<Args>::err(r.status());
      a.node_id = r.take_value();
      continue;
    }

    return wm::Result<Args>::err(wm::Status::invalid_argument("unknown flag: " + tok));
  }

  if (a.config_path.empty()) {
    return wm::Result<Args>::err(wm::Status::invalid_argument("--config is required"));
  }

  return wm::Result<Args>::ok(a);
}

void print_config_summary(const wm::Config& cfg) {
  const char* mode_str = (cfg.mode == wm::RunMode::kReplay) ? "replay" : "live";

  std::cout << "LiDAR World Model\n";
  std::cout << "-----------------\n";
  std::cout << "mode: " << mode_str << "\n";
  std::cout << "node_id: " << cfg.node_id << "\n";
  std::cout << "frames: lidar=" << cfg.frames.lidar_frame
            << " node=" << cfg.frames.node_frame
            << " site=" << cfg.frames.site_frame << "\n\n";

  std::cout << "baseline:\n";
  std::cout << "  warmup_s: " << ns_to_seconds(cfg.baseline.warmup_duration_ns) << "\n";
  std::cout << "  capture_s: " << ns_to_seconds(cfg.baseline.capture_duration_ns) << "\n\n";

  std::cout << "mapping:\n";
  std::cout << "  voxel_size_m: " << cfg.mapping.voxel_size_m << "\n";
  std::cout << "  block_size_vox: " << cfg.mapping.block_size_vox << "\n";
  std::cout << "  roi.min: [" << cfg.mapping.roi.min.x << ", " << cfg.mapping.roi.min.y << ", " << cfg.mapping.roi.min.z << "]\n";
  std::cout << "  roi.max: [" << cfg.mapping.roi.max.x << ", " << cfg.mapping.roi.max.y << ", " << cfg.mapping.roi.max.z << "]\n";
  std::cout << "  range_m: [" << cfg.mapping.min_range_m << ", " << cfg.mapping.max_range_m << "]\n";
  std::cout << "  integrate_hz: " << cfg.mapping.integrate_hz << "\n\n";

  std::cout << "budgets:\n";
  std::cout << "  max_points_per_sec: " << cfg.budgets.max_points_per_sec << "\n";
  std::cout << "  target_fps: " << cfg.budgets.target_fps << "\n";
  std::cout << "  downsample_voxel_m: " << cfg.budgets.downsample_voxel_m << "\n\n";

  std::cout << "change:\n";
  std::cout << "  persistence_s: " << ns_to_seconds(cfg.change.persistence_ns) << "\n";
  std::cout << "  min_cluster_volume_m3: " << cfg.change.min_cluster_volume_m3 << "\n";
  std::cout << "  min_aabb_edge_m: " << cfg.change.min_aabb_edge_m << "\n";
  std::cout << "  min_confidence: " << cfg.change.min_confidence << "\n";
  std::cout << "  prefer_site_frame: " << (cfg.change.prefer_site_frame ? "true" : "false") << "\n\n";

  if (cfg.mode == wm::RunMode::kReplay) {
    std::cout << "replay:\n";
    std::cout << "  dataset_path: " << cfg.replay.dataset_path << "\n";
    std::cout << "  time_scale: " << cfg.replay.time_scale << "\n";
    std::cout << "  start_offset_s: " << ns_to_seconds(cfg.replay.start_offset_ns) << "\n";
    std::cout << "  end_offset_s: " << ns_to_seconds(cfg.replay.end_offset_ns) << "\n";
    std::cout << "  loop: " << (cfg.replay.loop ? "true" : "false") << "\n\n";
  }

  std::cout << "output:\n";
  std::cout << "  out_dir: " << cfg.output.out_dir << "\n";
  std::cout << "  heartbeat_period_s: " << cfg.output.heartbeat_period_s << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  auto args_r = parse_args(argc, argv);
  if (!args_r.ok()) {
    // Special case: user asked for help.
    if (args_r.status().message() == "help") {
      print_usage(argv[0]);
      return 0;
    }

    std::cerr << "error: " << args_r.status().message() << "\n\n";
    print_usage(argv[0]);
    return 2;
  }
  Args args = args_r.take_value();

  auto cfg_r = wm::load_config(args.config_path);
  if (!cfg_r.ok()) {
    std::cerr << "error: failed to load config: " << cfg_r.status().message() << "\n";
    return 2;
  }
  wm::Config cfg = cfg_r.take_value();

  // Apply CLI overrides (kept intentionally minimal).
  if (!args.node_id.empty()) cfg.node_id = args.node_id;

  if (!args.mode.empty()) {
    auto m = parse_mode(args.mode);
    if (!m.ok()) {
      std::cerr << "error: " << m.status().message() << "\n";
      return 2;
    }
    cfg.mode = m.take_value();
  }

  if (!args.dataset_path.empty()) cfg.replay.dataset_path = args.dataset_path;
  if (!args.out_dir.empty()) cfg.output.out_dir = args.out_dir;

  // Re-validate after overrides.
  const wm::Status s = wm::validate_config(cfg);
  if (!s.ok()) {
    std::cerr << "error: invalid config after overrides: " << s.message() << "\n";
    return 2;
  }

    // Write a run header + one heartbeat so we can see output end-to-end.
  wm::JsonlEventSink sink;
  wm::RunInfo run;
  run.node_id = cfg.node_id;
  run.config_path = args.config_path;
  run.out_dir = cfg.output.out_dir;
  run.config_hash = "";       // wired up when repro_hash lands
  run.calibration_hash = "";  // wired up when repro_hash lands
  run.start_time = now_epoch_ns();

  {
    const wm::Status open_s = sink.open(run);
    if (!open_s.ok()) {
      std::cerr << "error: failed to open event sink: " << open_s.message() << "\n";
      return 2;
    }

    wm::Event hb;
    hb.type = "heartbeat";
    hb.timestamp = now_epoch_ns();
    hb.message = "wm_node config loaded";
    const wm::Status emit_s = sink.emit(hb);
    if (!emit_s.ok()) {
      std::cerr << "error: failed to emit heartbeat: " << emit_s.message() << "\n";
      return 2;
    }

    sink.flush();
    std::cout << "\nWrote events to: " << sink.path() << "\n";
  }

  // For now: just prove config load + deterministic overrides.
  print_config_summary(cfg);

  std::cout << "\nOK\n";
  return 0;
}
