// File: include/wm/adapters/frame_dir/frame_dir_source.hpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "wm/core/io/frame_source.hpp"

namespace wm {

struct FrameDirSourceConfig {
  std::string root_dir;               // e.g. data/replay/run_001
  std::string frames_subdir{"frames"}; // contains 000000.bin, 000001.bin, ...
  std::string timestamps_file{"timestamps_ns.txt"}; // one int64 ns per line (optional)
  bool loop{false};

  // If timestamps file is missing, we synthesize timestamps at this rate.
  double fallback_tick_hz{10.0};
};

class FrameDirSource final : public IFrameSource {
 public:
  explicit FrameDirSource(FrameDirSourceConfig cfg);

  // Call once after construction. Keeps ctor simple (no throwing / no implicit IO).
  Status open();

  Status next(Frame* out) override;
  Status reset() override;

  std::string name() const override { return "frame_dir"; }

 private:
  Status load_timestamps_if_present();
  static std::string join_path(const std::string& a, const std::string& b);
  static std::string frame_filename(std::size_t idx);

  FrameDirSourceConfig cfg_;
  bool opened_{false};

  std::vector<std::int64_t> timestamps_ns_;
  std::size_t idx_{0};

  std::int64_t fallback_period_ns_{100000000}; // 10 Hz
};

}  // namespace wm