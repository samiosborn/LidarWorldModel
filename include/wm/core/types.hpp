// include/wm/core/types.hpp
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace wm {

// -----------------------------
// Basic identifiers
// -----------------------------

using NodeId = std::string;     // e.g. "node_001"
using FrameName = std::string;  // e.g. "lidar", "node", "site"

// -----------------------------
// Time
// -----------------------------
// We keep timestamps as integer nanoseconds for determinism and portability.
// Interpretation (epoch vs dataset-relative) is defined by the adapter/dataset contract.

struct TimestampNs {
  std::int64_t ns = 0;

  constexpr bool operator==(const TimestampNs& other) const noexcept { return ns == other.ns; }
  constexpr bool operator!=(const TimestampNs& other) const noexcept { return ns != other.ns; }
  constexpr bool operator<(const TimestampNs& other) const noexcept { return ns < other.ns; }
  constexpr bool operator<=(const TimestampNs& other) const noexcept { return ns <= other.ns; }
  constexpr bool operator>(const TimestampNs& other) const noexcept { return ns > other.ns; }
  constexpr bool operator>=(const TimestampNs& other) const noexcept { return ns >= other.ns; }
};

// -----------------------------
// Geometry primitives
// -----------------------------

struct Vec3f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct AABB {
  Vec3f min;  // inclusive
  Vec3f max;  // upper bound (convention)

  [[nodiscard]] bool is_valid() const noexcept {
    return (min.x <= max.x) && (min.y <= max.y) && (min.z <= max.z);
  }

  [[nodiscard]] Vec3f size() const noexcept {
    return Vec3f{max.x - min.x, max.y - min.y, max.z - min.z};
  }

  [[nodiscard]] float volume() const noexcept {
    const auto s = size();
    return s.x * s.y * s.z;
  }
};

// A simple SE(3) transform represented as a 4x4 row-major matrix.
// Avoids Eigen early; we can swap internally later if we want.
struct TransformSE3 {
  std::array<float, 16> m = {
      1, 0, 0, 0,  //
      0, 1, 0, 0,  //
      0, 0, 1, 0,  //
      0, 0, 0, 1   //
  };

  static TransformSE3 identity() { return TransformSE3{}; }
};

// -----------------------------
// Point clouds
// -----------------------------

struct PointXYZI {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float intensity = 0.0f;
};

struct PointCloudFrame {
  TimestampNs timestamp;
  NodeId node_id;

  // Frame these points are expressed in (typically "lidar" for raw sensor output).
  FrameName frame = "lidar";

  // Monotonic frame index within the dataset (deterministic replay + debugging).
  std::uint64_t seq = 0;

  std::vector<PointXYZI> points;

  [[nodiscard]] std::size_t size() const noexcept { return points.size(); }
  [[nodiscard]] bool empty() const noexcept { return points.empty(); }
};

}  // namespace wm
