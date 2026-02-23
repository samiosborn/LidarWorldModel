// File: src/adapters/synth/synth_frame_source.cpp
#include "wm/adapters/synth/synth_frame_source.hpp"

#include <algorithm>
#include <cmath>

namespace wm {
namespace {

std::int64_t hz_to_period_ns(double hz) {
  if (hz <= 0.0) return 100000000;  // 10 Hz fallback
  const double ns = 1e9 / hz;
  return static_cast<std::int64_t>(std::llround(ns));
}

void append_box_surface_points(std::vector<PointXYZI>& pts,
                               const std::array<float, 3>& c,
                               const std::array<float, 3>& sz,
                               int grid_n) {
  grid_n = std::max(2, grid_n);
  const float hx = 0.5f * sz[0];
  const float hy = 0.5f * sz[1];
  const float hz = 0.5f * sz[2];

  const float x0 = c[0] - hx, x1 = c[0] + hx;
  const float y0 = c[1] - hy, y1 = c[1] + hy;
  const float z0 = c[2] - hz, z1 = c[2] + hz;

  // Sample 6 faces with a simple grid. This is not "real LiDAR", but it's deterministic and good
  // enough to exercise the pipeline.
  for (int i = 0; i < grid_n; ++i) {
    const float u = static_cast<float>(i) / static_cast<float>(grid_n - 1);
    const float x = x0 + u * (x1 - x0);
    const float y = y0 + u * (y1 - y0);
    for (int j = 0; j < grid_n; ++j) {
      const float v = static_cast<float>(j) / static_cast<float>(grid_n - 1);
      const float xx = x0 + v * (x1 - x0);
      const float yy = y0 + v * (y1 - y0);

      // z faces
      pts.push_back(PointXYZI{xx, yy, z0, 1.f});
      pts.push_back(PointXYZI{xx, yy, z1, 1.f});

      // x faces
      pts.push_back(PointXYZI{x0, y, z0 + v * (z1 - z0), 1.f});
      pts.push_back(PointXYZI{x1, y, z0 + v * (z1 - z0), 1.f});

      // y faces
      pts.push_back(PointXYZI{x, y0, z0 + v * (z1 - z0), 1.f});
      pts.push_back(PointXYZI{x, y1, z0 + v * (z1 - z0), 1.f});
    }
  }
}

}  // namespace

SynthFrameSource::SynthFrameSource(SynthSourceConfig cfg) : cfg_(std::move(cfg)) {
  tick_period_ns_ = hz_to_period_ns(cfg_.tick_hz);
}

Status SynthFrameSource::reset() {
  tick_ = 0;
  return Status::ok_status();
}

Status SynthFrameSource::next(Frame* out) {
  if (!out) return Status::invalid_argument("SynthFrameSource::next: out is null");

  const std::int64_t t_ns = tick_ * tick_period_ns_;
  out->t_ns = TimestampNs{t_ns};
  out->frame_id = "synth";

  out->points.clear();
  out->points.reserve(static_cast<std::size_t>(cfg_.floor_grid_n * cfg_.floor_grid_n) + 6u * 64u);

  // Floor grid
  const int n = std::max(2, cfg_.floor_grid_n);
  const float ext = std::max(0.5f, cfg_.floor_half_extent_m);
  for (int i = 0; i < n; ++i) {
    const float u = static_cast<float>(i) / static_cast<float>(n - 1);
    const float x = -ext + u * (2.f * ext);
    for (int j = 0; j < n; ++j) {
      const float v = static_cast<float>(j) / static_cast<float>(n - 1);
      const float y = -ext + v * (2.f * ext);
      out->points.push_back(PointXYZI{x, y, cfg_.floor_z_m, 0.2f});
    }
  }

  // Optional obstacle appears after obstacle_start_s, but absent during baseline.
  const double t_s = static_cast<double>(t_ns) * 1e-9;
  const bool in_baseline = (t_s < cfg_.baseline_s);
  const bool obstacle_on = cfg_.obstacle_enabled && !in_baseline && (t_s >= cfg_.obstacle_start_s);

  if (obstacle_on) {
    append_box_surface_points(out->points, cfg_.obstacle_center, cfg_.obstacle_size, /*grid_n=*/10);
  }

  ++tick_;
  return Status::ok_status();
}

}  // namespace wm