// File: include/wm/core/io/frame_source.hpp
#pragma once

#include <string>

#include "wm/core/io/frame.hpp"
#include "wm/core/status.hpp"

namespace wm {

class IFrameSource {
 public:
  virtual ~IFrameSource() = default;

  // Returns:
  //  - OK on success and fills `out`
  //  - out_of_range("eof") when no more frames
  //  - other error codes on failure
  virtual Status next(Frame* out) = 0;

  // Optional reset for looping replays.
  virtual Status reset() { return Status::unsupported("reset not supported"); }

  virtual std::string name() const = 0;
};

}  // namespace wm