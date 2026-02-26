// File: src/apps/wm_node/main.cpp
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "wm/adapters/frame_dir/frame_dir_source.hpp"
#include "wm/adapters/synth/synth_frame_source.hpp"
#include "wm/core/events/jsonl_event_sink.hpp"
#include "wm/core/io/frame_source.hpp"
#include "wm/core/model/node_runner.hpp"
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

std::unique_ptr<wm::FrameSource> make_source_from_config(const wm::Config& cfg) {
  if (cfg.input.type == "synth") {
    wm::SynthSourceConfig sc;
    sc.tick_hz = cfg.input.tick_hz;
    sc.seed = cfg.input.synth.seed;
    sc.num_points = cfg.input.synth.num_points;
    sc.enable_obstacle = cfg.input.synth.enable_obstacle;
    sc.obstacle_start_s = cfg.input.synth.obstacle_start_s;
    sc.moving_obstacle = cfg.input.synth.moving_obstacle;
    sc.obstacle_speed_mps = cfg.input.synth.obstacle_speed_mps;
    return std::make_unique<wm::SynthFrameSource>(sc);
  }

  if (cfg.input.type == "frame_dir") {
    wm::FrameDirSourceConfig dc;
    dc.path = cfg.input.frame_dir.path;
    dc.loop = cfg.input.frame_dir.loop;
    dc.fps = cfg.input.frame_dir.fps > 0.0 ? cfg.input.frame_dir.fps : cfg.input.tick_hz;
    return std::make_unique<wm::FrameDirSource>(dc);
  }

  return nullptr;
}

}  // namespace

int main(int argc, char** argv) {
  const Args args = parse_args(argc, argv);
  if (args.help || args.config_path.empty()) {
    print_usage();
    return args.help ? 0 : 2;
  }

  auto cfg_r = wm::load_config(args.config_path);
  if (!cfg_r.ok()) {
    std::cerr << cfg_r.status().message() << "\n";
    return 1;
  }
  wm::Config cfg = cfg_r.take_value();

  wm::NodeRunner runner(cfg, args.config_path);
  wm::JsonlEventSink sink;

  const wm::Status st_start = runner.start(sink);
  if (!st_start.ok()) {
    std::cerr << st_start.message() << "\n";
    return 2;
  }

  std::unique_ptr<wm::FrameSource> source = make_source_from_config(cfg);
  if (!source) {
    std::cerr << "Unknown input.type: " << cfg.input.type << "\n";
    return 2;
  }

  const wm::Status st_open = source->open();
  if (!st_open.ok()) {
    std::cerr << st_open.message() << "\n";
    return 2;
  }

  // Ensure we always close/flush cleanly.
  struct Guard {
    wm::NodeRunner& r;
    wm::JsonlEventSink& s;
    wm::FrameSource& src;
    ~Guard() {
      src.close();
      r.stop(s);
    }
  } guard{runner, sink, *source};

  std::cout << "Events: " << sink.path() << " (latest: " << sink.latest_path() << ")\n";
  std::cout << "Input: " << cfg.input.type << "  tick_hz=" << cfg.input.tick_hz
            << "  heartbeat_every_s=" << cfg.input.heartbeat_every_s << "\n\n";

  using clock = std::chrono::steady_clock;

  const auto tick_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / cfg.input.tick_hz));

  const auto t_start = clock::now();
  auto next_tick = t_start + tick_period;
  auto last_hb = t_start - std::chrono::seconds(cfg.input.heartbeat_every_s > 0
                                                     ? cfg.input.heartbeat_every_s
                                                     : 0);

  std::int64_t tick_count = 0;
  bool had_error = false;

  while (true) {
    const auto now = clock::now();

    if (cfg.input.max_ticks > 0 && tick_count >= cfg.input.max_ticks) {
      (void)runner.emit_event(sink, "shutdown", "max_ticks reached");
      (void)sink.flush();
      break;
    }

    if (cfg.input.max_run_s > 0.0) {
      const auto max_d = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(cfg.input.max_run_s));
      if (now - t_start >= max_d) {
        (void)runner.emit_event(sink, "shutdown", "max_runtime reached");
        (void)sink.flush();
        break;
      }
    }

    if (cfg.input.heartbeat_every_s > 0 &&
        now - last_hb >= std::chrono::seconds(cfg.input.heartbeat_every_s)) {
      last_hb = now;
      const wm::Status st = runner.emit_heartbeat(sink, "alive tick=" + std::to_string(tick_count));
      if (!st.ok()) {
        std::cerr << st.message() << "\n";
        had_error = true;
        break;
      }
    }

    auto frame_r = source->next();
    if (frame_r.ok()) {
      wm::Frame frame = frame_r.take_value();
      const wm::Status st = runner.emit_event(
          sink, "frame_stats",
          "frame_id=" + frame.frame_id + " num_points=" + std::to_string(frame.points.size()));
      if (!st.ok()) {
        std::cerr << st.message() << "\n";
        had_error = true;
        break;
      }
    } else if (frame_r.status().code() == wm::Status::Code::kOutOfRange) {
      (void)runner.emit_event(sink, "input_eof", "input source reached end");
      (void)sink.flush();
      break;
    } else {
      std::cerr << frame_r.status().message() << "\n";
      had_error = true;
      break;
    }

    const wm::Status st_flush = sink.flush();
    if (!st_flush.ok()) {
      std::cerr << st_flush.message() << "\n";
      had_error = true;
      break;
    }

    ++tick_count;

    const auto after = clock::now();
    if (after < next_tick) {
      std::this_thread::sleep_until(next_tick);
      next_tick += tick_period;
    } else {
      next_tick = after + tick_period;
    }
  }

  if (had_error) return 2;
  std::cout << "OK\n";
  return 0;
}
