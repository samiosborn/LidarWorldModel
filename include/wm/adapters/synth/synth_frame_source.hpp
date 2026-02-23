// File: include/wm/adapters/synth/synth_frame_source.hpp
#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "wm/core/io/frame_source.hpp"

namespace wm {

struct SynthSourceConfig {
  double tick_hz{10.0};

  // Scene sampling density (kept deliberately simple).
  int floor_grid_n{40};          // floor grid is N x N points
  float floor_half_extent_m{8.f}; // grid covers [-extent, +extent] in x/y
  float floor_z_m{0.f};

  // Baseline / obstacle timing.
  double baseline_s{5.0};        // obstacle absent during baseline
  double obstacle_start_s{8.0};  // obstacle appears after this time
  bool obstacle_enabled{true};

  // Axis-aligned box obstacle in "node" frame.
  std::array<float, 3> obstacle_center{2.0f, 0.0f, 0.5f};
  std::array<float, 3> obstacle_size{1.0f, 1.0f, 1.0f};

  std::uint32_t seed{1};
};

class SynthFrameSource final : public IFrameSource {
 public:
  explicit SynthFrameSource(SynthSourceConfig cfg);

  Status next(Frame* out) override;
  Status reset() override;

  std::string name() const override { return "synth"; }

 private:
  std::int64_t tick_period_ns_{100000000}; // 10 Hz default
  std::int64_t tick_{0};

  SynthSourceConfig cfg_;
};

}  // namespace wm