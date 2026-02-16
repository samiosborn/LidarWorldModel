# LiDAR World Model

Nexus is a deployable fixed 3D LiDAR edge node for factories. It maintains a local 3D world model and emits **events** (not raw point clouds). The MVP focuses on **actionable geometric change detection** with deterministic replay and config-driven fidelity.

## Product goal

- Maintain a high-fidelity geometric twin (target **cm-level**, configurable).
- Detect meaningful changes:
  - e.g. **“new obstacle”** if change is **> X size** and persists **> T seconds** (both in config).
- Emit minimal payload events:
  - for MVP: **axis-aligned 3D bounding boxes (AABB)**.
- Produce operational logs and descriptions like:
  - “something moved/appeared here” (based on geometric deltas; richer semantics later).

## Baseline

MVP uses a controlled baseline workflow:

1. **Baseline capture** (“learn normal” for N seconds/minutes).
2. Detect changes against the baseline, using persistence filters.
3. **Re-baseline** is a deliberate operation when the environment changes permanently.

Later versions can evolve to a slowly updating background model that absorbs persistent structure and ignores transient motion.

---

## Core design principles

- **Modular, testable pipeline** with deterministic replay as a first-class feature.
- Strict separation of:
  - **Core algorithms** (platform-agnostic)
  - **Adapters** (Isaac log replay now, ROS2/drivers later)
- Multi-node ready from day 1:
  - `node_id`
  - explicit coordinate frames (`lidar`, `node`, `site`)
  - stored transforms (`T_node_lidar`, placeholder `T_site_node`)
  - versioned calibration + **config hashing** (events/logs always include a reproducibility fingerprint)

---

## Repository structure

nexus/
README.md
LICENSE
CMakeLists.txt
cmake/
third_party/

configs/
mapping.yaml
budgets.yaml
nuisance.yaml
change_detection.yaml
calibration.yaml
site.yaml
profiles/
jetson_orin.yaml
desktop_dev.yaml

schemas/
events/
obstacle_added.schema.json
obstacle_removed.schema.json
calibration/
extrinsics.schema.json

include/nexus/
core/
geom/
mapping/
change/
io/
util/

src/
core/
mapping/
change/
nuisance/
model/
events/
metrics/
adapters/
replay/
isaac/
ros2/
apps/
nexus_node/
tools/
cli/

tools/
convert_isaac_logs/
dataset_tools/
viz/

data/
datasets/
golden_runs/
no_change/
add_obstacle/
remove_obstacle/
occlusion/
expected_outputs/

tests/
unit/
integration/
regression/
fixtures/

benchmarks/
throughput/
jetson_budget/

docs/
architecture.md
config_reference.md
coordinate_frames.md
dataset_format.md

scripts/
run_replay.sh
run_golden_suite.sh

docker/
toolchains/
.github/workflows/


### What goes where

- `src/core/`  
  All platform-agnostic logic:
  - voxel occupancy world model (free/occupied/unknown)
  - filtering (ground removal, outliers, masking, exclusion zones)
  - change detection (clustering, min volume, persistence time)
  - event building (AABB, confidence, duration)
  - metrics (false positives/min, latency, min detectable size)

- `src/adapters/`  
  Anything that touches external systems:
  - file replay reader/parser (MVP)
  - Isaac log conversion/parsing
  - ROS2 (later): subscriptions, TF, QoS, message types

- `src/apps/`  
  Executables:
  - `nexus_node`: runs continuously, builds world model, detects change, emits events/logs
  - `tools/`: re-baselining, map inspection, config hash reporting, dataset utilities

- `configs/`  
  YAML configs that control fidelity, budgets, nuisance handling, change thresholds, calibration.

- `data/datasets/golden_runs/`  
  Deterministic replay datasets for regression and metrics tracking.

---

## World model representation (MVP)

**Voxel occupancy map** with 3 states:

- `free`
- `occupied`
- `unknown` (unobserved; important for occlusion reasoning)

Fidelity is controlled via config:
- voxel size (cm-level target, adjustable)
- ROI / bounds
- update rates, decimation policies

---

## Event output (MVP)

Start with:
- `obstacle_added`
- optionally `obstacle_removed`

Each event includes:
- `timestamp`, `node_id`
- AABB (in `site` frame if available; otherwise `node`)
- size/volume, confidence, persistence duration
- optional minimal metadata for movement heuristics (e.g. overlap with previous AABB)

Schemas live in `schemas/events/` and should remain backwards compatible.

---

## Coordinate frames & transforms

We treat frames as explicit and versioned.

- `lidar` frame: sensor origin
- `node` frame: physical node mounting frame
- `site` frame: factory/site coordinate frame (optional in MVP)

Transforms:
- `T_node_lidar`: required (extrinsics)
- `T_site_node`: optional now; required for multi-node site alignment later

See `docs/coordinate_frames.md`.

---

## Config-driven operation

Configs are split by concern (and can be layered):

- Mapping / fidelity: voxel resolution, ROI, integration rate, decimation
- Throughput budgets: max points/sec, target FPS, CPU/GPU targets, downsampling policy
- Noise & nuisance: intensity/range gating, outlier removal, ground removal, unstable-voxel masking
- Exclusion zones: ignore volumes (static) and masks (dynamic)
- Change detection: min cluster size/volume, persistence time `T`, confidence thresholds
- Calibration: extrinsics, versioning

**Reproducibility:** every run computes a **config + calibration hash** and includes it in logs and emitted events.

---

## Build (WSL / Ubuntu)

### Prerequisites
- CMake (recommend 3.22+)
- A C++20 compiler (GCC 11+ or Clang 14+ recommended)
- Optional but common deps:
  - Eigen (math/geometry)
  - fmt + spdlog (logging)
  - yaml-cpp (configs)

### Build commands
```bash
# From repo root
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
