// File: src/adapters/synth/synth_frame_source.cpp
#include "wm/adapters/synth/synth_frame_source.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <utility>

namespace wm {
namespace {

std::int64_t hz_to_period_ns(double hz) {
  if (hz <= 0.0) return 100000000;
  const double ns = 1e9 / hz;
  return static_cast<std::int64_t>(std::llround(ns));
}

}  // namespace

SynthFrameSource::SynthFrameSource(SynthSourceConfig cfg) : cfg_(std::move(cfg)) {
  tick_period_ns_ = hz_to_period_ns(cfg_.tick_hz);
}

Status SynthFrameSource::open() {
  if (cfg_.num_points <= 0) {
    return Status::invalid_argument("SynthFrameSource: num_points must be > 0");
  }
  tick_ = 0;
  build_static_scene();
  opened_ = true;
  return Status::ok_status();
}

Result<Frame> SynthFrameSource::next() {
  if (!opened_) {
    return Result<Frame>::err(Status::invalid_argument("SynthFrameSource::next: not opened"));
  }

  Frame out;
  const std::int64_t t_ns = tick_ * tick_period_ns_;
  out.t_ns = TimestampNs{t_ns};
  out.frame_id = "synth_" + std::to_string(tick_);
  out.points = static_points_;

  const double t_s = static_cast<double>(t_ns) * 1e-9;
  if (cfg_.enable_obstacle && t_s >= cfg_.obstacle_start_s) {
    append_obstacle_points(out.points, t_s);
  }

  ++tick_;
  return Result<Frame>::ok(std::move(out));
}

void SynthFrameSource::close() {
  opened_ = false;
  tick_ = 0;
  static_points_.clear();
}

void SynthFrameSource::build_static_scene() {
  static_points_.clear();
  static_points_.reserve(static_cast<std::size_t>(cfg_.num_points));

  std::mt19937 rng(cfg_.seed);
  std::uniform_real_distribution<float> xy_dist(-8.0f, 8.0f);

  for (int i = 0; i < cfg_.num_points; ++i) {
    static_points_.push_back(PointXYZI{xy_dist(rng), xy_dist(rng), 0.0f, 0.2f});
  }
}

void SynthFrameSource::append_obstacle_points(std::vector<PointXYZI>& points, double t_s) const {
  constexpr int kGrid = 8;
  constexpr float kHalfSize = 0.5f;
  float cx = 2.0f;
  const float cy = 0.0f;
  const float cz = 0.5f;

  if (cfg_.moving_obstacle) {
    const double dt_s = std::max(0.0, t_s - cfg_.obstacle_start_s);
    cx += static_cast<float>(cfg_.obstacle_speed_mps * dt_s);
  }

  const float x0 = cx - kHalfSize;
  const float x1 = cx + kHalfSize;
  const float y0 = cy - kHalfSize;
  const float y1 = cy + kHalfSize;
  const float z0 = cz - kHalfSize;
  const float z1 = cz + kHalfSize;

  for (int i = 0; i < kGrid; ++i) {
    const float u = static_cast<float>(i) / static_cast<float>(kGrid - 1);
    const float x = x0 + u * (x1 - x0);
    const float y = y0 + u * (y1 - y0);
    for (int j = 0; j < kGrid; ++j) {
      const float v = static_cast<float>(j) / static_cast<float>(kGrid - 1);
      const float xx = x0 + v * (x1 - x0);
      const float yy = y0 + v * (y1 - y0);

      points.push_back(PointXYZI{xx, yy, z0, 1.0f});
      points.push_back(PointXYZI{xx, yy, z1, 1.0f});
      points.push_back(PointXYZI{x0, y, z0 + v * (z1 - z0), 1.0f});
      points.push_back(PointXYZI{x1, y, z0 + v * (z1 - z0), 1.0f});
      points.push_back(PointXYZI{x, y0, z0 + v * (z1 - z0), 1.0f});
      points.push_back(PointXYZI{x, y1, z0 + v * (z1 - z0), 1.0f});
    }
  }
}

}  // namespace wm
