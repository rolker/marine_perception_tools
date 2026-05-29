# Agent Guide: marine_perception_tools

> Operator-station tools for marine perception; hosts the `sea_surface_tuner`
> offline bag-replay + segmentation→costmap tuning app.

## Workflow

**When this repo is checked out as part of a
[ROS 2 Agent Workspace](https://github.com/rolker/ros2_agent_workspace)**,
workflow rules (worktree vs. field mode, branch naming, etc.) are defined in the
workspace `AGENTS.md`. Determine the active mode before editing, from the
workspace root:

```bash
.agent/scripts/field_mode.sh --describe layers/main/ui_ws/src/marine_perception_tools
```

**Standalone use** (this repo cloned alone): only this repo's own conventions
apply.

## Package Inventory

| Package | Language | Description |
|---------|----------|-------------|
| `marine_perception_tools` | C++ (Qt5) | Builds `sea_surface_tuner` — a forward-camera bag-replay + costmap re-tuning viewer (Milestone A of [#1](https://github.com/rolker/marine_perception_tools/issues/1)). |

## Repository Layout

```
marine_perception_tools/
├── src/
│   ├── bag_loader.{hpp,cpp}    # two-pass bag read → forward-camera PreparedFrames (TF, camera model)
│   ├── resim_engine.{hpp,cpp}  # OccupancyBuffer wrapper: replay via accumulate_frame, re-sim, render
│   ├── cv_qt.hpp               # cv::Mat (rgb8/BGR) → QImage helper
│   ├── main_window.{hpp,cpp}   # MainWindow: panes, scrubber, data-driven param dock
│   └── main.cpp                # CLI parse → load bag → engine + window
├── test/
│   └── test_resim_engine.cpp   # GTest: determinism, re-sim equivalence, clear, bounds-reject
├── CMakeLists.txt              # ament_cmake; tuner_core library + Qt exe + gtest
├── package.xml
├── .github/workflows/ci.yml
└── .pre-commit-config.yaml
```

The `tuner_core` library (`bag_loader` + `resim_engine`) is deliberately Qt-free
so the re-sim logic is unit-testable without a display; `sea_surface_tuner` links
it plus Qt.

## Build & Test

```bash
# From the ui_ws layer workspace (lower layers — incl. sea_surface_segmentation — sourced)
colcon build --packages-select marine_perception_tools --symlink-install
# Testing requires setup.bash in the same shell
source ../../../.agent/scripts/setup.bash && colcon test --packages-select marine_perception_tools && colcon test-result --verbose
```

## Common Pitfalls

- **Qt5, not Qt6.** Jazzy's operator station is Qt5-only. The CMake is written
  version-agnostically (`Qt${QT_VERSION_MAJOR}`) so the eventual Qt6 bump (next
  ROS LTS) is mechanical — but do not assume Qt6 headers/APIs are available here.
- **ui_ws is operator-station-only** — this package and its Qt dependency must
  never become a boat-side build dependency.
- The tuner is a **standalone Qt app**, not an rqt plugin; it is not bound to
  rqt's Qt version, only to the Jazzy system Qt.
- It reuses the real algorithm from `sea_surface_segmentation`'s exported
  headers ([unh_marine_perception#23](https://github.com/rolker/unh_marine_perception/issues/23)) —
  `occupancy_buffer.hpp`, `segments_projection.hpp`, `occupancy_accumulator.hpp`
  (header-only; the rendering palette is *copied* from the tool's non-exported
  `render_new`, so it can drift cosmetically).
- **Stale `sea_surface_segmentation` install gotcha**: those exported headers
  only appear in the install tree after `sea_surface_segmentation` is
  *rebuilt* post-#23. An install predating #24 has an empty
  `include/sea_surface_segmentation/`, and the tuner then fails to compile.
  Rebuild `sea_surface_segmentation` (or `make build`) if headers aren't found.
- **`res`/`half_extent` are construction-time geometry** — they size the
  `OccupancyBuffer` and are CLI-only, not live dock knobs (changing them needs a
  new engine). The live dock holds only `setParams`-able / projection knobs.
