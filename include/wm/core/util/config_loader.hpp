// include/wm/core/util/config_loader.hpp
#pragma once

#include <string>

#include "wm/core/config.hpp"
#include "wm/core/status.hpp"

namespace wm {

// Loads a YAML config file (supports optional `includes:` for layering).
// - Includes are loaded first (in order), then overridden by the main file.
// - Relative include paths are resolved relative to the including file.
//
// Returns a fully populated Config with defaults applied + validated.
Result<Config> load_config(const std::string& path);

}  // namespace wm
