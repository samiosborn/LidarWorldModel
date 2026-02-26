// File: include/wm/core/io/frame_source.hpp
#pragma once

#include "wm/core/io/frame.hpp"
#include "wm/core/status.hpp"

namespace wm {

class FrameSource {
 public:
  virtual ~FrameSource() = default;
  virtual Status open() = 0;
  virtual Result<Frame> next() = 0;
  virtual void close() = 0;
};

}  // namespace wm
