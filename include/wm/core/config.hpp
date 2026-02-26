// include/wm/core/config.hpp
#pragma once

#include <cstdint>
#include <string>

#include "wm/core/status.hpp"
#include "wm/core/types.hpp"

namespace wm {

// Units policy:
// - Distances in metres
// - Angles in radians (when we add them)
// - Time durations in nanoseconds (int64) for determinism

using DurationNs = std::int64_t;

constexpr DurationNs seconds_to_ns(double seconds) {
  return static_cast<DurationNs>(seconds * 1'000'000'000.0);
}

// -----------------------------
// High-level run mode
// -----------------------------
enum class RunMode {
  kReplay,
  kLive,  // placeholder for later
};

// -----------------------------
// Frames / transforms (minimal config surface)
// -----------------------------
struct FramesConfig {
  FrameName lidar_frame = "lidar";
  FrameName node_frame  = "node";
  FrameName site_frame  = "site";  // optional until site alignment exists
};

// Optional now, required later for multi-node alignment.
struct CalibrationConfig {
  // Path to calibration YAML (or a directory/ID later).
  std::string calibration_path;

  // Extrinsics: transform from lidar -> node (T_node_lidar).
  // If identity here, you're effectively saying lidar_frame == node_frame.
  TransformSE3 T_node_lidar = TransformSE3::identity();

  // Placeholder for later: transform from node -> site (T_site_node).
  // Leave identity if unknown.
  TransformSE3 T_site_node = TransformSE3::identity();

  // A version string you control (also ends up hashed).
  std::string calibration_version = "dev";
};

// -----------------------------
// Baseline capture
// -----------------------------
struct BaselineConfig {
  // How long we "learn normal" before freezing baseline.
  DurationNs capture_duration_ns = seconds_to_ns(30.0);

  // Optional: ignore the first few seconds while the sensor settles.
  DurationNs warmup_duration_ns = seconds_to_ns(0.0);
};

// -----------------------------
// Mapping / fidelity
// -----------------------------
struct RoiConfig {
  // Axis-aligned ROI in node frame unless stated otherwise.
  // Keep it explicit; no implicit "infinite" bounds.
  Vec3f min = Vec3f{-10.0f, -10.0f, -2.0f};
  Vec3f max = Vec3f{+10.0f, +10.0f, +5.0f};
};

struct MappingConfig {
  // Sparse voxel resolution.
  float voxel_size_m = 0.02f;  // 2 cm default

  // Sparse hashed block size (e.g. 8 means 8x8x8 voxels per block).
  int block_size_vox = 8;

  RoiConfig roi;

  // Input gating (very cheap nuisance filtering).
  float min_range_m = 0.2f;
  float max_range_m = 50.0f;

  // Keep intensity for later nuisance handling; doesn't affect core mapping yet.
  bool use_intensity = true;

  // Integration rate target (replay can exceed; live will aim for this).
  int integrate_hz = 10;
};

// -----------------------------
// Budgets / throttling
// -----------------------------
struct BudgetsConfig {
  // Hard cap: if exceeded, we must downsample / decimate.
  std::int64_t max_points_per_sec = 2'000'000;

  // Soft target.
  int target_fps = 10;

  // When over budget, voxel-grid downsample input points at this size (metres).
  float downsample_voxel_m = 0.03f;
};

// -----------------------------
// Change detection
// -----------------------------
struct ChangeDetectionConfig {
  // Change must persist this long before emitting an event.
  DurationNs persistence_ns = seconds_to_ns(2.0);

  // Minimum cluster volume in m^3 (filters tiny noise blobs).
  float min_cluster_volume_m3 = 0.01f;  // e.g. 10 litres

  // Minimum AABB edge length (metres).
  float min_aabb_edge_m = 0.10f;

  // Confidence threshold for emission (0..1). Definition is internal.
  float min_confidence = 0.6f;

  // Event frame preference:
  // - if true and site transform is non-trivial, emit in site_frame
  // - else emit in node_frame
  bool prefer_site_frame = true;
};

// -----------------------------
// Replay input (deterministic)
// -----------------------------
struct ReplayConfig {
  // Dataset root directory (contains a manifest + frame files).
  std::string dataset_path;

  // Playback speed: 1.0 = real-time according to timestamps, 0 = as fast as possible.
  // For deterministic regression: prefer 0 (as fast as possible) + fixed ordering.
  double time_scale = 0.0;

  // Optional trimming (nanoseconds, dataset time).
  // Set to 0 to disable.
  DurationNs start_offset_ns = 0;
  DurationNs end_offset_ns = 0;

  // Loop dataset (useful for soak testing).
  bool loop = false;
};

// -----------------------------
// Input source (Milestone 1)
// -----------------------------
struct InputSynthConfig {
  std::uint32_t seed = 1;
  int num_points = 1600;
  bool enable_obstacle = true;
  double obstacle_start_s = 8.0;
  bool moving_obstacle = false;
  float obstacle_speed_mps = 0.25f;
};

struct InputFrameDirConfig {
  std::string path;
  bool loop = false;
  // If <= 0, wm_node uses input.tick_hz for generated frame timestamps.
  double fps = 0.0;
};

struct InputConfig {
  std::string type = "synth";  // synth | frame_dir
  double tick_hz = 10.0;
  int heartbeat_every_s = 5;   // 0 disables
  std::int64_t max_ticks = 0;  // 0 disables
  double max_run_s = 0.0;      // 0 disables

  InputSynthConfig synth;
  InputFrameDirConfig frame_dir;
};

// -----------------------------
// Output (events + logs)
// -----------------------------
struct OutputConfig {
  // Where to write event JSONL and run metadata.
  std::string out_dir = "out";

  // Emit a heartbeat event every N seconds (0 disables).
  int heartbeat_period_s = 5;
};

// -----------------------------
// Root config
// -----------------------------
struct Config {
  RunMode mode = RunMode::kReplay;

  NodeId node_id = "node_001";
  FramesConfig frames;
  CalibrationConfig calibration;

  BaselineConfig baseline;
  MappingConfig mapping;
  BudgetsConfig budgets;
  ChangeDetectionConfig change;

  ReplayConfig replay;
  InputConfig input;
  OutputConfig output;
};

// Minimal validation (keep it strict; fail early).
inline Status validate_config(const Config& cfg) {
  if (cfg.node_id.empty()) {
    return Status::invalid_argument("node_id must not be empty");
  }
  if (cfg.mapping.voxel_size_m <= 0.0f) {
    return Status::invalid_argument("mapping.voxel_size_m must be > 0");
  }
  if (cfg.mapping.block_size_vox <= 0) {
    return Status::invalid_argument("mapping.block_size_vox must be > 0");
  }
  if (!AABB{cfg.mapping.roi.min, cfg.mapping.roi.max}.is_valid()) {
    return Status::invalid_argument("mapping.roi must be a valid AABB (min <= max)");
  }
  if (cfg.change.persistence_ns < 0) {
    return Status::invalid_argument("change.persistence_ns must be >= 0");
  }
  if (cfg.budgets.max_points_per_sec <= 0) {
    return Status::invalid_argument("budgets.max_points_per_sec must be > 0");
  }
  if (cfg.budgets.target_fps <= 0) {
    return Status::invalid_argument("budgets.target_fps must be > 0");
  }
  if (cfg.output.out_dir.empty()) {
    return Status::invalid_argument("output.out_dir must not be empty");
  }
  if (cfg.input.tick_hz <= 0.0) {
    return Status::invalid_argument("input.tick_hz must be > 0");
  }
  if (cfg.input.heartbeat_every_s < 0) {
    return Status::invalid_argument("input.heartbeat_every_s must be >= 0");
  }
  if (cfg.input.max_ticks < 0) {
    return Status::invalid_argument("input.max_ticks must be >= 0");
  }
  if (cfg.input.max_run_s < 0.0) {
    return Status::invalid_argument("input.max_run_s must be >= 0");
  }
  if (cfg.input.synth.num_points <= 0) {
    return Status::invalid_argument("input.synth.num_points must be > 0");
  }
  if (cfg.input.type != "synth" && cfg.input.type != "frame_dir") {
    return Status::invalid_argument("input.type must be 'synth' or 'frame_dir'");
  }
  if (cfg.input.type == "frame_dir" && cfg.input.frame_dir.path.empty()) {
    return Status::invalid_argument("input.frame_dir.path must not be empty for frame_dir input");
  }
  return Status::ok_status();
}

}  // namespace wm
