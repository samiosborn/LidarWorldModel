// File: include/wm/core/io/frame.hpp
#pragma once

#include <string>
#include <vector>

#include "wm/core/types.hpp"  // TimestampNs, PointXYZI

namespace wm {

struct Frame {
  // Logical time for the frame. For synth: ticks since start. For replay: dataset time or ticks.
  TimestampNs t_ns{0};
  std::string frame_id;
  std::vector<PointXYZI> points;
};

}  // namespace wm