// File: include/wm/core/io/frame.hpp
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "wm/core/events/event_sink.hpp"  // TimestampNs

namespace wm {

struct PointXYZI {
  float x{0.f};
  float y{0.f};
  float z{0.f};
  float intensity{0.f};
};

struct Frame {
  // Logical time for the frame. For synth: ticks since start. For replay: dataset time or ticks.
  TimestampNs t_ns{0};
  std::string frame_id;
  std::vector<PointXYZI> points;
};

}  // namespace wm