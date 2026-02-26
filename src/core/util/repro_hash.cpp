// File: src/core/util/repro_hash.cpp
#include "wm/core/util/repro_hash.hpp"

#include <bit>
#include <cstdint>
#include <string>

namespace wm {
namespace {

// FNV-1a 64-bit. Not cryptographic. Exactly what we want for fast, stable fingerprints.
struct Fnv1a64 {
  std::uint64_t h = 1469598103934665603ull;

  void add_bytes(const void* data, std::size_t n) {
    const auto* p = static_cast<const std::uint8_t*>(data);
    for (std::size_t i = 0; i < n; ++i) {
      h ^= static_cast<std::uint64_t>(p[i]);
      h *= 1099511628211ull;
    }
  }

  void add_u64(std::uint64_t v) { add_bytes(&v, sizeof(v)); }
  void add_i64(std::int64_t v)  { add_bytes(&v, sizeof(v)); }
  void add_u32(std::uint32_t v) { add_bytes(&v, sizeof(v)); }
  void add_i32(std::int32_t v)  { add_bytes(&v, sizeof(v)); }

  void add_bool(bool v) {
    const std::uint8_t b = v ? 1u : 0u;
    add_bytes(&b, sizeof(b));
  }

  void add_string(const std::string& s) {
    // Include length so ("ab","c") != ("a","bc") in concatenations.
    add_u64(static_cast<std::uint64_t>(s.size()));
    add_bytes(s.data(), s.size());
  }

  void add_float(float v) {
    const std::uint32_t bits = std::bit_cast<std::uint32_t>(v);
    add_u32(bits);
  }

  void add_double(double v) {
    const std::uint64_t bits = std::bit_cast<std::uint64_t>(v);
    add_u64(bits);
  }
};

std::string to_hex(std::uint64_t v) {
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out(16, '0');
  for (int i = 15; i >= 0; --i) {
    out[static_cast<std::size_t>(i)] = kHex[v & 0xF];
    v >>= 4;
  }
  return out;
}

void add_vec3(Fnv1a64& h, const Vec3f& v) {
  h.add_float(v.x);
  h.add_float(v.y);
  h.add_float(v.z);
}

void add_aabb(Fnv1a64& h, const AABB& a) {
  // If AABB is invalid, still hash whatever is in there: invalid configs should fingerprint too.
  add_vec3(h, a.min);
  add_vec3(h, a.max);
}

void add_transform(Fnv1a64& h, const TransformSE3& T) {
  // Transform is stored as a 4x4 float matrix in row-major order (see config_loader).
  for (std::size_t i = 0; i < T.m.size(); ++i) h.add_float(T.m[i]);
}

void add_mode(Fnv1a64& h, RunMode m) {
  const std::int32_t v = (m == RunMode::kReplay) ? 1 : 2;
  h.add_i32(v);
}

}  // namespace

std::string compute_calibration_hash(const CalibrationConfig& calib) {
  Fnv1a64 h;

  // This is the payload that affects geometry alignment.
  h.add_string(calib.calibration_path);
  h.add_string(calib.calibration_version);
  add_transform(h, calib.T_node_lidar);
  add_transform(h, calib.T_site_node);

  return to_hex(h.h);
}

std::string compute_config_hash(const Config& cfg) {
  Fnv1a64 h;

  // High-level.
  add_mode(h, cfg.mode);
  h.add_string(cfg.node_id);

  // Frames.
  h.add_string(cfg.frames.lidar_frame);
  h.add_string(cfg.frames.node_frame);
  h.add_string(cfg.frames.site_frame);

  // Calibration (hash the real payload, but also include version/path at top-level).
  h.add_string(cfg.calibration.calibration_path);
  h.add_string(cfg.calibration.calibration_version);
  add_transform(h, cfg.calibration.T_node_lidar);
  add_transform(h, cfg.calibration.T_site_node);

  // Baseline.
  h.add_i64(cfg.baseline.capture_duration_ns);
  h.add_i64(cfg.baseline.warmup_duration_ns);

  // Mapping.
  h.add_float(cfg.mapping.voxel_size_m);
  h.add_i32(cfg.mapping.block_size_vox);
  add_aabb(h, AABB{cfg.mapping.roi.min, cfg.mapping.roi.max});
  h.add_float(cfg.mapping.min_range_m);
  h.add_float(cfg.mapping.max_range_m);
  h.add_bool(cfg.mapping.use_intensity);
  h.add_i32(cfg.mapping.integrate_hz);

  // Budgets.
  h.add_i64(cfg.budgets.max_points_per_sec);
  h.add_i32(cfg.budgets.target_fps);
  h.add_float(cfg.budgets.downsample_voxel_m);

  // Change detection.
  h.add_i64(cfg.change.persistence_ns);
  h.add_float(cfg.change.min_cluster_volume_m3);
  h.add_float(cfg.change.min_aabb_edge_m);
  h.add_float(cfg.change.min_confidence);
  h.add_bool(cfg.change.prefer_site_frame);

  // Replay.
  h.add_string(cfg.replay.dataset_path);
  h.add_double(cfg.replay.time_scale);
  h.add_i64(cfg.replay.start_offset_ns);
  h.add_i64(cfg.replay.end_offset_ns);
  h.add_bool(cfg.replay.loop);

  // Input.
  h.add_string(cfg.input.type);
  h.add_double(cfg.input.tick_hz);
  h.add_i32(cfg.input.heartbeat_every_s);
  h.add_i64(cfg.input.max_ticks);
  h.add_double(cfg.input.max_run_s);

  h.add_u32(cfg.input.synth.seed);
  h.add_i32(cfg.input.synth.num_points);
  h.add_bool(cfg.input.synth.enable_obstacle);
  h.add_double(cfg.input.synth.obstacle_start_s);
  h.add_bool(cfg.input.synth.moving_obstacle);
  h.add_float(cfg.input.synth.obstacle_speed_mps);

  h.add_string(cfg.input.frame_dir.path);
  h.add_bool(cfg.input.frame_dir.loop);
  h.add_double(cfg.input.frame_dir.fps);

  // Output.
  h.add_string(cfg.output.out_dir);
  h.add_i32(cfg.output.heartbeat_period_s);

  return to_hex(h.h);
}

}  // namespace wm
