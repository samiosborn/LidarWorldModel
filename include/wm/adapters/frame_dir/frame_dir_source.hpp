// File: include/wm/adapters/frame_dir/frame_dir_source.hpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "wm/core/io/frame_source.hpp"

namespace wm {

struct FrameDirSourceConfig {
  // Directory containing frame files.
  // Format: one ".bin" file per frame, each file packed as float32 x,y,z,intensity per point.
  // Files are consumed in lexicographic filename order for deterministic playback.
  std::string path;
  bool loop{false};
  // If <= 0, caller pacing is used but timestamps are still synthesized at 10 Hz.
  double fps{0.0};
};

class FrameDirSource final : public FrameSource {
 public:
  explicit FrameDirSource(FrameDirSourceConfig cfg);
  ~FrameDirSource() override { close(); }

  Status open() override;
  Result<Frame> next() override;
  void close() override;

 private:
  Status load_file_list();
  Result<Frame> read_frame(const std::string& path, const std::string& frame_id);

  FrameDirSourceConfig cfg_;
  bool opened_{false};

  std::vector<std::string> frame_paths_;
  std::vector<std::string> frame_ids_;
  std::size_t idx_{0};
  std::int64_t emitted_{0};

  std::int64_t frame_period_ns_{100000000};
};

}  // namespace wm
