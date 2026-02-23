// File: src/apps/wm_node/main.cpp
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>

#include "wm/adapters/frame_dir/frame_dir_source.hpp"
#include "wm/adapters/synth/synth_frame_source.hpp"
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
    a.help = true;
    return a;
  }
  return a;
}

void print_usage() {
  std::cout << "wm_node\n"
            << "  --config <path>\n";
}

struct RuntimeParams {
  double tick_hz{10.0};
  double heartbeat_period_s{5.0};

  // How often to emit a lightweight "frame_received" event.
  // 1 = every tick, 10 (at 10 Hz) = once per second, 0 = never.
  int frame_event_every_n{10};

  // 0 means run forever (for synth). For replay, it will also stop at EOF unless loop=true.
  double max_runtime_s{0.0};
};

struct SourceParams {
  std::string type{"synth"}; // synth | frame_dir
  bool loop{true};

  // frame_dir
  std::string frame_dir_root;

  // synth
  double baseline_s{5.0};
  double obstacle_start_s{8.0};
};

RuntimeParams load_runtime_params(const std::string& yaml_path) {
  RuntimeParams p;

  YAML::Node n;
  try {
    n = YAML::LoadFile(yaml_path);
  } catch (...) {
    return p;  // defaults
  }

  const auto r = n["runtime"];
  if (!r) return p;

  if (r["tick_hz"]) p.tick_hz = r["tick_hz"].as<double>(p.tick_hz);
  if (r["heartbeat_period_s"]) p.heartbeat_period_s = r["heartbeat_period_s"].as<double>(p.heartbeat_period_s);
  if (r["frame_event_every_n"]) p.frame_event_every_n = r["frame_event_every_n"].as<int>(p.frame_event_every_n);
  if (r["max_runtime_s"]) p.max_runtime_s = r["max_runtime_s"].as<double>(p.max_runtime_s);

  if (p.tick_hz <= 0.0) p.tick_hz = 10.0;
  if (p.heartbeat_period_s <= 0.0) p.heartbeat_period_s = 5.0;
  if (p.frame_event_every_n < 0) p.frame_event_every_n = 0;

  // Reasonable default: once per second if user didn't set it.
  if (p.frame_event_every_n == 0) {
    // keep disabled if explicitly 0
  } else if (p.frame_event_every_n == 10 && p.tick_hz != 10.0) {
    p.frame_event_every_n = static_cast<int>(p.tick_hz);
    if (p.frame_event_every_n < 1) p.frame_event_every_n = 1;
  }

  return p;
}

SourceParams load_source_params(const std::string& yaml_path) {
  SourceParams p;

  YAML::Node n;
  try {
    n = YAML::LoadFile(yaml_path);
  } catch (...) {
    return p;  // defaults
  }

  const auto s = n["source"];
  if (!s) return p;

  if (s["type"]) p.type = s["type"].as<std::string>(p.type);
  if (s["loop"]) p.loop = s["loop"].as<bool>(p.loop);

  const auto fd = s["frame_dir"];
  if (fd && fd["root_dir"]) p.frame_dir_root = fd["root_dir"].as<std::string>("");

  const auto sy = s["synth"];
  if (sy) {
    if (sy["baseline_s"]) p.baseline_s = sy["baseline_s"].as<double>(p.baseline_s);
    if (sy["obstacle_start_s"]) p.obstacle_start_s = sy["obstacle_start_s"].as<double>(p.obstacle_start_s);
  }

  return p;
}

}  // namespace

int main(int argc, char** argv) {
  const Args args = parse_args(argc, argv);
  if (args.help || args.config_path.empty()) {
    print_usage();
    return args.help ? 0 : 2;
  }

  const wm::Result<wm::Config> cfg_r = wm::load_config(args.config_path);
  if (!cfg_r.ok()) {
    std::cerr << cfg_r.status().message() << "\n";
    return 1;
  }
  wm::Config cfg = cfg_r.take_value();

  const RuntimeParams rt = load_runtime_params(args.config_path);
  const SourceParams sp = load_source_params(args.config_path);

  std::unique_ptr<wm::IFrameSource> source;

  if (sp.type == "synth") {
    wm::SynthSourceConfig sc;
    sc.tick_hz = rt.tick_hz;
    sc.baseline_s = sp.baseline_s;
    sc.obstacle_start_s = sp.obstacle_start_s;
    source = std::make_unique<wm::SynthFrameSource>(sc);
  } else if (sp.type == "frame_dir") {
    wm::FrameDirSourceConfig fdc;
    fdc.root_dir = sp.frame_dir_root;
    fdc.loop = sp.loop;
    fdc.fallback_tick_hz = rt.tick_hz;

    auto* ptr = new wm::FrameDirSource(fdc);
    const wm::Status st = ptr->open();
    if (!st.ok()) {
      std::cerr << st.message() << "\n";
      delete ptr;
      return 2;
    }
    source.reset(ptr);
  } else {
    std::cerr << "Unknown source.type: " << sp.type << "\n";
    return 2;
  }

  wm::NodeRunner runner(cfg, args.config_path);
  wm::JsonlEventSink sink;

  const wm::Status st_start = runner.start(sink);
  if (!st_start.ok()) {
    std::cerr << st_start.message() << "\n";
    return 2;
  }

  // Ensure we always close/flush cleanly.
  struct Guard {
    wm::NodeRunner& r;
    wm::JsonlEventSink& s;
    ~Guard() { r.stop(s); }
  } guard{runner, sink};

  std::cout << "Events: " << sink.path() << " (latest: " << sink.latest_path() << ")\n";
  std::cout << "Source: " << source->name() << "  tick_hz=" << rt.tick_hz
            << "  heartbeat_s=" << rt.heartbeat_period_s << "\n\n";

  using clock = std::chrono::steady_clock;

  const auto tick_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rt.tick_hz));

  const auto heartbeat_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(rt.heartbeat_period_s));

  const auto t_start = clock::now();
  auto next_tick = t_start + tick_period;
  auto last_hb = t_start;

  std::int64_t tick_count = 0;
  std::int64_t frame_count = 0;

  while (true) {
    const auto now = clock::now();

    // Optional runtime limit.
    if (rt.max_runtime_s > 0.0) {
      const auto max_d = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(rt.max_runtime_s));
      if (now - t_start >= max_d) {
        (void)runner.emit_event(sink, "shutdown", "max_runtime reached");
        (void)sink.flush();
        break;
      }
    }

    // Heartbeat.
    if (now - last_hb >= heartbeat_period) {
      last_hb = now;
      const wm::Status st = runner.emit_heartbeat(
          sink, "alive tick=" + std::to_string(tick_count) + " frames=" + std::to_string(frame_count));
      if (!st.ok()) {
        std::cerr << st.message() << "\n";
        break;
      }
      (void)sink.flush();
    }

    // Pull next frame (if available).
    wm::Frame frame;
    const wm::Status st = source->next(&frame);
    if (st.ok()) {
      ++frame_count;
      if (rt.frame_event_every_n > 0 && (tick_count % rt.frame_event_every_n == 0)) {
        const wm::Status st2 = runner.emit_event(
            sink, "frame_received",
            "frame_id=" + frame.frame_id + " points=" + std::to_string(frame.points.size()));
        if (!st2.ok()) {
          std::cerr << st2.message() << "\n";
          break;
        }
      }
    } else if (st.code() == wm::Status::Code::kOutOfRange) {
      if (sp.loop) {
        const wm::Status st_reset = source->reset();
        if (!st_reset.ok()) {
          std::cerr << st_reset.message() << "\n";
          break;
        }
        (void)runner.emit_event(sink, "replay_reset", "looping to start");
        (void)sink.flush();
      } else {
        (void)runner.emit_event(sink, "replay_eof", "end of replay");
        (void)sink.flush();
        break;
      }
    } else {
      std::cerr << st.message() << "\n";
      break;
    }

    ++tick_count;

    // Tick pacing.
    const auto after = clock::now();
    if (after < next_tick) {
      std::this_thread::sleep_until(next_tick);
      next_tick += tick_period;
    } else {
      // We overran: don't spiral. Resync.
      next_tick = after + tick_period;
    }
  }

  std::cout << "OK\n";
  return 0;
}