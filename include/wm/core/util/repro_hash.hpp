// File: include/wm/core/util/repro_hash.hpp
#pragma once

#include <string>

#include "wm/core/config.hpp"

namespace wm {

// Hash the full runtime config (including replay/output paths if they're in Config).
// Goal: if the run changes, this hash should change.
std::string compute_config_hash(const Config& cfg);

// Hash only the calibration payload (extrinsics + versioning).
// Goal: calibration changes should be obvious in logs.
std::string compute_calibration_hash(const CalibrationConfig& calib);

}  // namespace wm