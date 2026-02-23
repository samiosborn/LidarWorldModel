// File: src/adapters/frame_dir/frame_dir_source.cpp
#include "wm/adapters/frame_dir/frame_dir_source.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>

namespace wm {
namespace {

std::int64_t hz_to_period_ns(double hz) {
  if (hz <= 0.0) return 100000000;  // 10 Hz
  return static_cast<std::int64_t>(1e9 / hz);
}

}  // namespace

FrameDirSource::FrameDirSource(FrameDirSourceConfig cfg) : cfg_(std::move(cfg)) {
  fallback_period_ns_ = hz_to_period_ns(cfg_.fallback_tick_hz);
}

Status FrameDirSource::open() {
  if (cfg_.root_dir.empty()) return Status::invalid_argument("FrameDirSource: root_dir is empty");
  Status st = load_timestamps_if_present();
  if (!st.ok()) return st;
  opened_ = true;
  idx_ = 0;
  return Status::ok_status();
}

Status FrameDirSource::reset() {
  if (!opened_) return Status::invalid_argument("FrameDirSource::reset: not opened");
  idx_ = 0;
  return Status::ok_status();
}

std::string FrameDirSource::join_path(const std::string& a, const std::string& b) {
  namespace fs = std::filesystem;
  return (fs::path(a) / fs::path(b)).string();
}

std::string FrameDirSource::frame_filename(std::size_t idx) {
  std::ostringstream ss;
  ss.width(6);
  ss.fill('0');
  ss << idx << ".bin";
  return ss.str();
}

Status FrameDirSource::load_timestamps_if_present() {
  timestamps_ns_.clear();

  const std::string ts_path = join_path(cfg_.root_dir, cfg_.timestamps_file);
  std::ifstream f(ts_path);
  if (!f.is_open()) {
    // Optional file: OK to not have it.
    return Status::ok_status();
  }

  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    try {
      const std::int64_t t = std::stoll(line);
      timestamps_ns_.push_back(t);
    } catch (...) {
      return Status::parse_error("FrameDirSource: failed parsing timestamps_ns.txt");
    }
  }

  return Status::ok_status();
}

Status FrameDirSource::next(Frame* out) {
  if (!out) return Status::invalid_argument("FrameDirSource::next: out is null");
  if (!opened_) return Status::invalid_argument("FrameDirSource::next: not opened");

  namespace fs = std::filesystem;

  const std::string frames_dir = join_path(cfg_.root_dir, cfg_.frames_subdir);
  const std::string fname = frame_filename(idx_);
  const std::string path = join_path(frames_dir, fname);

  std::error_code ec;
  if (!fs::exists(path, ec)) {
    return Status::out_of_range("eof");
  }

  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) return Status::io_error("FrameDirSource: failed to open frame file");

  f.seekg(0, std::ios::end);
  const std::streamoff nbytes = f.tellg();
  f.seekg(0, std::ios::beg);

  if (nbytes <= 0 || (nbytes % (3 * static_cast<std::streamoff>(sizeof(float))) != 0)) {
    return Status::corrupt_data("FrameDirSource: frame file size not multiple of 3*float");
  }

  const std::size_t npts =
      static_cast<std::size_t>(nbytes / (3 * static_cast<std::streamoff>(sizeof(float))));

  std::vector<float> buf(3 * npts);
  f.read(reinterpret_cast<char*>(buf.data()),
         static_cast<std::streamsize>(buf.size() * sizeof(float)));
  if (!f) return Status::io_error("FrameDirSource: short read");

  out->points.clear();
  out->points.reserve(npts);
  for (std::size_t i = 0; i < npts; ++i) {
    const float x = buf[3 * i + 0];
    const float y = buf[3 * i + 1];
    const float z = buf[3 * i + 2];
    out->points.push_back(PointXYZI{x, y, z, 0.f});
  }

  const std::int64_t t_ns =
      (idx_ < timestamps_ns_.size()) ? timestamps_ns_[idx_] : static_cast<std::int64_t>(idx_) * fallback_period_ns_;

  out->t_ns = TimestampNs{t_ns};
  out->frame_id = fname;

  ++idx_;
  return Status::ok_status();
}

}  // namespace wm