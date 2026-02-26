// File: include/wm/adapters/synth/synth_frame_source.hpp
#pragma once

#include <cstdint>
#include <vector>

#include "wm/core/io/frame_source.hpp"

namespace wm {

struct SynthSourceConfig {
  double tick_hz{10.0};
  std::uint32_t seed{1};
  int num_points{1600};

  bool enable_obstacle{true};
  double obstacle_start_s{8.0};
  bool moving_obstacle{false};
  float obstacle_speed_mps{0.25f};
};

class SynthFrameSource final : public FrameSource {
 public:
  explicit SynthFrameSource(SynthSourceConfig cfg);
  ~SynthFrameSource() override { close(); }

  Status open() override;
  Result<Frame> next() override;
  void close() override;

 private:
  void append_obstacle_points(std::vector<PointXYZI>& points, double t_s) const;
  void build_static_scene();

  SynthSourceConfig cfg_;
  bool opened_{false};

  std::int64_t tick_period_ns_{100000000};
  std::int64_t tick_{0};

  std::vector<PointXYZI> static_points_;
};

}  // namespace wm
