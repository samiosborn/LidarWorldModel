// File: src/apps/wm_node/main.cpp
#include <iostream>
#include <string>

#include "wm/core/events/jsonl_event_sink.hpp"
#include "wm/core/model/node_runner.hpp"
#include "wm/core/status.hpp"
#include "wm/core/util/config_loader.hpp"

namespace {

struct Args {
  std::string config_path;
  bool help{false};
};

Args parse_args(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string s = argv[i];
    if (s == "--help" || s == "-h") {
      a.help = true;
      return a;
    }
    if (s == "--config" && i + 1 < argc) {
      a.config_path = argv[++i];
      continue;
    }
    // Keep it strict for now: unknown args are a user mistake.
    a.help = true;
    return a;
  }
  return a;
}

void print_usage() {
  std::cout << "wm_node\n"
            << "  --config <path>\n";
}

}  // namespace

int main(int argc, char** argv) {
  const Args args = parse_args(argc, argv);
  if (args.help || args.config_path.empty()) {
    print_usage();
    return args.help ? 0 : 2;
  }

  // Load config.
  const wm::Result<wm::Config> cfg_r = wm::load_config(args.config_path);
  if (!cfg_r.ok()) {
    std::cerr << cfg_r.status().message() << "\n";
    return 1;
  }

  wm::Config cfg = *cfg_r;

  // Single place owns time + run lifecycle.
  wm::NodeRunner runner(cfg, args.config_path);

  // Sink owns event file layout.
  wm::JsonlEventSink sink;

  {
    const wm::Status st = runner.start(sink);
    if (!st.ok()) {
      std::cerr << st.message() << "\n";
      return 2;
    }
  }

  // CLI prints once. Libraries stay quiet.
  std::cout << "Events: " << sink.path() << " (latest: " << sink.latest_path() << ")\n\n";

  // Minimal smoke test: emit a heartbeat so we can see JSONL wiring.
  {
    const wm::Status st = runner.emit_heartbeat(sink, "wm_node config loaded");
    if (!st.ok()) {
      std::cerr << st.message() << "\n";
      runner.stop(sink);
      return 2;
    }
  }

  runner.stop(sink);

  std::cout << "OK\n";
  return 0;
}