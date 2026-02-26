// File: src/adapters/frame_dir/frame_dir_source.cpp
#include "wm/adapters/frame_dir/frame_dir_source.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <utility>

namespace wm {
namespace {

std::int64_t hz_to_period_ns(double hz) {
  if (hz <= 0.0) return 100000000;
  return static_cast<std::int64_t>(1e9 / hz);
}

}  // namespace

FrameDirSource::FrameDirSource(FrameDirSourceConfig cfg) : cfg_(std::move(cfg)) {
  frame_period_ns_ = hz_to_period_ns(cfg_.fps > 0.0 ? cfg_.fps : 10.0);
}

Status FrameDirSource::open() {
  if (cfg_.path.empty()) return Status::invalid_argument("FrameDirSource: path is empty");
  close();
  WM_RETURN_IF_ERROR(load_file_list());
  opened_ = true;
  idx_ = 0;
  emitted_ = 0;
  return Status::ok_status();
}

Status FrameDirSource::load_file_list() {
  namespace fs = std::filesystem;

  frame_paths_.clear();
  frame_ids_.clear();

  std::error_code ec;
  if (!fs::exists(cfg_.path, ec)) {
    return Status::not_found("FrameDirSource: directory not found: " + cfg_.path);
  }
  if (!fs::is_directory(cfg_.path, ec)) {
    return Status::invalid_argument("FrameDirSource: path is not a directory: " + cfg_.path);
  }

  std::vector<std::pair<std::string, std::string>> entries;
  for (const auto& it : fs::directory_iterator(cfg_.path, ec)) {
    if (ec) {
      return Status::io_error("FrameDirSource: failed listing directory: " + cfg_.path);
    }
    if (!it.is_regular_file(ec)) continue;
    if (it.path().extension() != ".bin") continue;
    entries.push_back(std::make_pair(it.path().filename().string(), it.path().string()));
  }

  std::sort(entries.begin(), entries.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  for (const auto& e : entries) {
    frame_ids_.push_back(e.first);
    frame_paths_.push_back(e.second);
  }

  if (frame_paths_.empty()) {
    return Status::not_found("FrameDirSource: no .bin files found in " + cfg_.path);
  }
  return Status::ok_status();
}

Result<Frame> FrameDirSource::read_frame(const std::string& path, const std::string& frame_id) {
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) {
    return Result<Frame>::err(Status::io_error("FrameDirSource: failed to open " + path));
  }

  f.seekg(0, std::ios::end);
  const std::streamoff nbytes = f.tellg();
  f.seekg(0, std::ios::beg);

  const std::streamoff stride = 4 * static_cast<std::streamoff>(sizeof(float));
  if (nbytes <= 0 || (nbytes % stride) != 0) {
    return Result<Frame>::err(Status::corrupt_data(
        "FrameDirSource: frame file size not multiple of 4*float"));
  }

  const std::size_t npts = static_cast<std::size_t>(nbytes / stride);

  std::vector<float> buf(4 * npts);
  f.read(reinterpret_cast<char*>(buf.data()),
         static_cast<std::streamsize>(buf.size() * sizeof(float)));
  if (!f) return Result<Frame>::err(Status::io_error("FrameDirSource: short read"));

  Frame out;
  out.frame_id = frame_id;
  out.points.reserve(npts);
  for (std::size_t i = 0; i < npts; ++i) {
    const float x = buf[4 * i + 0];
    const float y = buf[4 * i + 1];
    const float z = buf[4 * i + 2];
    const float intensity = buf[4 * i + 3];
    out.points.push_back(PointXYZI{x, y, z, intensity});
  }

  return Result<Frame>::ok(std::move(out));
}

Result<Frame> FrameDirSource::next() {
  if (!opened_) {
    return Result<Frame>::err(Status::invalid_argument("FrameDirSource::next: not opened"));
  }
  if (frame_paths_.empty()) {
    return Result<Frame>::err(Status::out_of_range("eof"));
  }

  if (idx_ >= frame_paths_.size()) {
    if (!cfg_.loop) {
      return Result<Frame>::err(Status::out_of_range("eof"));
    }
    idx_ = 0;
  }

  auto frame_r = read_frame(frame_paths_[idx_], frame_ids_[idx_]);
  if (!frame_r.ok()) return frame_r;

  Frame frame = frame_r.take_value();
  frame.t_ns = TimestampNs{emitted_ * frame_period_ns_};

  ++idx_;
  ++emitted_;
  return Result<Frame>::ok(std::move(frame));
}

void FrameDirSource::close() {
  opened_ = false;
  frame_paths_.clear();
  frame_ids_.clear();
  idx_ = 0;
  emitted_ = 0;
}

}  // namespace wm
