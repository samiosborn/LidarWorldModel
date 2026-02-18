// src/core/util/config_loader.cpp
#include "wm/core/config_loader.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace wm {
namespace fs = std::filesystem;

static std::string to_lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

static bool is_map(const YAML::Node& n) { return n && n.IsMap(); }
static bool is_scalar(const YAML::Node& n) { return n && n.IsScalar(); }

// Recursive merge: maps merge keys; scalars/sequences override.
static YAML::Node merge_yaml(const YAML::Node& base, const YAML::Node& override_) {
  if (!base) return override_;
  if (!override_) return base;

  if (base.IsMap() && override_.IsMap()) {
    YAML::Node out = YAML::Clone(base);
    for (auto it : override_) {
      const auto key = it.first.as<std::string>();
      const auto val = it.second;
      if (out[key]) out[key] = merge_yaml(out[key], val);
      else out[key] = val;
    }
    return out;
  }

  // For scalars, sequences, etc., override completely.
  return override_;
}

template <typename T>
static void maybe_set(const YAML::Node& n, const char* key, T& out) {
  if (!n || !n[key]) return;
  out = n[key].as<T>();
}

static Result<TransformSE3> parse_transform4x4(const YAML::Node& n) {
  if (!n) return Result<TransformSE3>::err(Status::invalid_argument("transform node missing"));
  if (!n.IsSequence() || n.size() != 4) {
    return Result<TransformSE3>::err(Status::invalid_argument("transform must be a 4x4 sequence"));
  }

  TransformSE3 T;
  for (std::size_t r = 0; r < 4; ++r) {
    const auto row = n[r];
    if (!row.IsSequence() || row.size() != 4) {
      return Result<TransformSE3>::err(Status::invalid_argument("transform must be a 4x4 sequence"));
    }
    for (std::size_t c = 0; c < 4; ++c) {
      T.m[r * 4 + c] = row[c].as<float>();
    }
  }
  return Result<TransformSE3>::ok(T);
}

static Result<YAML::Node> load_yaml_file(const fs::path& path) {
  try {
    if (!fs::exists(path)) {
      return Result<YAML::Node>::err(Status::not_found("config not found: " + path.string()));
    }
    return Result<YAML::Node>::ok(YAML::LoadFile(path.string()));
  } catch (const YAML::Exception& e) {
    return Result<YAML::Node>::err(Status::parse_error("YAML parse error in " + path.string() + ": " + e.what()));
  } catch (const std::exception& e) {
    return Result<YAML::Node>::err(Status::io_error("failed to load " + path.string() + ": " + e.what()));
  }
}

static Result<YAML::Node> load_with_includes(const fs::path& path) {
  auto root_r = load_yaml_file(path);
  if (!root_r.ok()) return Result<YAML::Node>::err(root_r.status());
  YAML::Node root = root_r.take_value();

  YAML::Node merged;  // empty
  const fs::path dir = path.parent_path();

  // Optional top-level includes: ["a.yaml", "b.yaml"]
  if (root["includes"]) {
    const YAML::Node inc = root["includes"];
    if (!inc.IsSequence()) {
      return Result<YAML::Node>::err(Status::invalid_argument("includes must be a YAML sequence"));
    }

    for (std::size_t i = 0; i < inc.size(); ++i) {
      const auto rel = inc[i].as<std::string>();
      const fs::path child = fs::path(rel).is_absolute() ? fs::path(rel) : (dir / rel);
      auto child_r = load_with_includes(child);  // recursive
      if (!child_r.ok()) return Result<YAML::Node>::err(child_r.status());
      merged = merge_yaml(merged, child_r.take_value());
    }
  }

  // Finally override with this file's contents (excluding includes itself).
  if (root["includes"]) root.remove("includes");
  merged = merge_yaml(merged, root);
  return Result<YAML::Node>::ok(merged);
}

static Result<RunMode> parse_run_mode(const YAML::Node& n) {
  if (!n) return Result<RunMode>::ok(RunMode::kReplay);
  if (!is_scalar(n)) return Result<RunMode>::err(Status::invalid_argument("mode must be a string"));
  const auto s = to_lower(n.as<std::string>());
  if (s == "replay") return Result<RunMode>::ok(RunMode::kReplay);
  if (s == "live") return Result<RunMode>::ok(RunMode::kLive);
  return Result<RunMode>::err(Status::invalid_argument("unknown mode: " + s));
}

Result<Config> load_config(const std::string& path_str) {
  const fs::path path = fs::path(path_str);

  auto yaml_r = load_with_includes(path);
  if (!yaml_r.ok()) return Result<Config>::err(yaml_r.status());
  const YAML::Node y = yaml_r.take_value();

  Config cfg;  // defaults

  // --- high-level
  if (y["mode"]) {
    auto m = parse_run_mode(y["mode"]);
    if (!m.ok()) return Result<Config>::err(m.status());
    cfg.mode = m.take_value();
  }
  maybe_set(y, "node_id", cfg.node_id);

  // --- frames
  if (is_map(y["frames"])) {
    const auto f = y["frames"];
    maybe_set(f, "lidar_frame", cfg.frames.lidar_frame);
    maybe_set(f, "node_frame",  cfg.frames.node_frame);
    maybe_set(f, "site_frame",  cfg.frames.site_frame);
  }

  // --- calibration (minimal for now)
  if (is_map(y["calibration"])) {
    const auto c = y["calibration"];
    maybe_set(c, "calibration_path", cfg.calibration.calibration_path);
    maybe_set(c, "calibration_version", cfg.calibration.calibration_version);

    if (c["T_node_lidar"]) {
      auto t = parse_transform4x4(c["T_node_lidar"]);
      if (!t.ok()) return Result<Config>::err(t.status());
      cfg.calibration.T_node_lidar = t.take_value();
    }
    if (c["T_site_node"]) {
      auto t = parse_transform4x4(c["T_site_node"]);
      if (!t.ok()) return Result<Config>::err(t.status());
      cfg.calibration.T_site_node = t.take_value();
    }
  }

  // --- baseline
  if (is_map(y["baseline"])) {
    const auto b = y["baseline"];
    if (b["capture_duration_s"]) cfg.baseline.capture_duration_ns = seconds_to_ns(b["capture_duration_s"].as<double>());
    if (b["warmup_duration_s"])  cfg.baseline.warmup_duration_ns  = seconds_to_ns(b["warmup_duration_s"].as<double>());
  }

  // --- mapping
  if (is_map(y["mapping"])) {
    const auto m = y["mapping"];
    maybe_set(m, "voxel_size_m", cfg.mapping.voxel_size_m);
    maybe_set(m, "block_size_vox", cfg.mapping.block_size_vox);
    maybe_set(m, "min_range_m", cfg.mapping.min_range_m);
    maybe_set(m, "max_range_m", cfg.mapping.max_range_m);
    maybe_set(m, "use_intensity", cfg.mapping.use_intensity);
    maybe_set(m, "integrate_hz", cfg.mapping.integrate_hz);

    if (is_map(m["roi"])) {
      const auto r = m["roi"];
      if (is_map(r["min"])) {
        const auto mn = r["min"];
        maybe_set(mn, "x", cfg.mapping.roi.min.x);
        maybe_set(mn, "y", cfg.mapping.roi.min.y);
        maybe_set(mn, "z", cfg.mapping.roi.min.z);
      }
      if (is_map(r["max"])) {
        const auto mx = r["max"];
        maybe_set(mx, "x", cfg.mapping.roi.max.x);
        maybe_set(mx, "y", cfg.mapping.roi.max.y);
        maybe_set(mx, "z", cfg.mapping.roi.max.z);
      }
    }
  }

  // --- budgets
  if (is_map(y["budgets"])) {
    const auto b = y["budgets"];
    maybe_set(b, "max_points_per_sec", cfg.budgets.max_points_per_sec);
    maybe_set(b, "target_fps", cfg.budgets.target_fps);
    maybe_set(b, "downsample_voxel_m", cfg.budgets.downsample_voxel_m);
  }

  // --- change detection
  if (is_map(y["change"])) {
    const auto c = y["change"];
    if (c["persistence_s"]) cfg.change.persistence_ns = seconds_to_ns(c["persistence_s"].as<double>());
    maybe_set(c, "min_cluster_volume_m3", cfg.change.min_cluster_volume_m3);
    maybe_set(c, "min_aabb_edge_m", cfg.change.min_aabb_edge_m);
    maybe_set(c, "min_confidence", cfg.change.min_confidence);
    maybe_set(c, "prefer_site_frame", cfg.change.prefer_site_frame);
  }

  // --- replay
  if (is_map(y["replay"])) {
    const auto r = y["replay"];
    maybe_set(r, "dataset_path", cfg.replay.dataset_path);
    maybe_set(r, "time_scale", cfg.replay.time_scale);
    if (r["start_offset_s"]) cfg.replay.start_offset_ns = seconds_to_ns(r["start_offset_s"].as<double>());
    if (r["end_offset_s"])   cfg.replay.end_offset_ns   = seconds_to_ns(r["end_offset_s"].as<double>());
    maybe_set(r, "loop", cfg.replay.loop);
  }

  // --- output
  if (is_map(y["output"])) {
    const auto o = y["output"];
    maybe_set(o, "out_dir", cfg.output.out_dir);
    maybe_set(o, "heartbeat_period_s", cfg.output.heartbeat_period_s);
  }

  // Final validation (fail early).
  const Status s = validate_config(cfg);
  if (!s.ok()) return Result<Config>::err(s);

  return Result<Config>::ok(cfg);
}

}  // namespace wm
