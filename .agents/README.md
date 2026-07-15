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
| `marine_perception_tools` | C++ (Qt5) | Builds `sea_surface_tuner` — a 4-camera bag-replay + costmap re-tuning viewer (menu/File→Open with windowed buffering, fused costmap, raw+compressed segmentation, H.265 RGB decode, recorded-vs-regenerated compare, Apply/Reset param dock) ([#1](https://github.com/rolker/marine_perception_tools/issues/1)). |
| *(same package)* | C++ (Qt5) | Also builds `sidescan_target_viewer` — the offline georeferenced sidescan/MBES viewer + target tracker ([#7](https://github.com/rolker/marine_perception_tools/issues/7)/[#8](https://github.com/rolker/marine_perception_tools/issues/8)): geo coverage pane, waterfall, water-column echogram, 3D point cloud, contact marking. CLI: `sidescan_target_viewer [bag] [--start T --end T] [--index survey_index.db [--stores DIR]]` — the optional time window (UNIX ns or ISO-8601, UTC assumed) cues the scrub to that interval once indexing completes (jump-to-pass from `survey_index_query`, [#17](https://github.com/rolker/marine_perception_tools/issues/17)). `--index` opens the **survey overview window** ([#19](https://github.com/rolker/marine_perception_tools/issues/19), explorer stage 2): store-tile basemap (GGGS GeoTIFFs via `marine_tiled_raster_store`; `--stores` defaults to `<index dir>/bathymetry/survey`), click → pass list from the index → double-click opens a viewer cued to that pass. Note: opening a bag always runs a full-bag *metadata* scan (TF + cumulative distance need the whole recording); only sample data is window-read, so the cue does not add a whole-bag sample read. |

## Repository Layout

```
marine_perception_tools/
├── src/
│   ├── bag_loader.{hpp,cpp}    # BagSession: one full scan (TF cache, models, bounds) + loadWindow(start,end) windowed reads; load_bag() one-shot wrapper. 4-camera merged PreparedFrames + RGB (H.265) + recorded costmap
│   ├── buffer_policy.hpp       # pure plan_buffer(): given scrub time t, decide engine replay span + I/O-cache extent + reload action (no Qt/ROS)
│   ├── resim_engine.{hpp,cpp}  # OccupancyBuffer wrapper: fuse all cameras, re-sim, render (+ boat heading marker); checkpoint store for cheap backward seek; seekToStampDisplayOnly (images w/o warm-up); batched setParams; boat_yaw
│   ├── cv_qt.hpp               # cv::Mat (rgb8/BGR) → QImage helper
│   ├── main_window.{hpp,cpp}   # MainWindow: menu/File→Open, splitter pane grid, whole-bag scrubber, ASYNC buffer manager (QtConcurrent two-stage load: images then costmap), async Apply, render/rescale split, param tooltips
│   └── main.cpp                # CLI parse (+ --probe headless) → window → openBag
├── test/
│   ├── test_resim_engine.cpp   # GTest: determinism, re-sim equivalence, clear, bounds-reject, multi-camera fusion, checkpoint rewind==fresh-replay, batched setParams
│   └── test_buffer_policy.cpp  # GTest: pure span-policy boundaries (fresh/extend/far-jump/retention-trim/clamps)
├── CMakeLists.txt              # ament_cmake; tuner_core library + Qt exe + 2 gtests
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
- **`max_pool_bins` is pinned, not exposed.** `unh_marine_perception#26` added a
  per-cell max-pool to `project_observations_inverse` gated by a `max_pool_bins`
  arg (default **true**); the deployed `SeaSurfaceLayer` exposes it as a ROS
  param. The tuner reaches the algorithm via `accumulate_frame`, which doesn't
  forward it → the tuner is hardwired to the `true` default. This matches the
  boat *while the layer keeps the default*, and the tuner's dock does NOT yet let
  you A/B it. If the field flips `max_pool_bins` off, expose it as a dock toggle
  (it's a bool, so the double-typed knob table needs a small extension) to keep
  parity. Verified build + fidelity compatible with #26/#27 (2026-06-01).
- **Stale `sea_surface_segmentation` install gotcha**: those exported headers
  only appear in the install tree after `sea_surface_segmentation` is
  *rebuilt* post-#23. An install predating #24 has an empty
  `include/sea_surface_segmentation/`, and the tuner then fails to compile.
  Rebuild `sea_surface_segmentation` (or `make build`) if headers aren't found.
- **`res`/`half_extent` are construction-time geometry** — they size the
  `OccupancyBuffer` and are CLI-only, not live dock knobs (changing them needs a
  new engine). The live dock holds only `setParams`-able / projection knobs.
- **Windowed File→Open, not whole-bag.** `MainWindow` keeps one `BagSession`
  (the single scan) and loads only a window around the scrub point via
  `plan_buffer` → `loadWindow`; an unbounded whole-bag load OOMs on long
  4-camera bags. The engine accumulates exactly `[t−integration, t]`, and the
  costmap at a given frame is a pure function of `(frame, params, window-start)`.
  Re-sim cost is `frames × n²` cells (`n = 2·half_extent/res`): a full window
  replay is ~tens of seconds, so **never re-sim per scrub step or per keystroke.**
  The `ReSimEngine` checkpoint store makes *backward* seeks cheap (restore nearest
  snapshot + replay the short tail — bit-identical to a fresh replay because an
  `OccupancyBuffer` copy captures the decay clock); a parameter change clears the
  checkpoints, so the dock batches edits behind **Apply** (one re-sim) rather than
  re-simulating on every `valueChanged`.
- **`--probe` does the one-shot `load_bag`, not the windowed read** — so a
  probe with no `--start-s/--end-s` clamp buffers the entire bag and can OOM.
  Always clamp manual probes.
- **Window loads + Apply run on a worker thread** (`QtConcurrent::run` +
  `QFutureWatcher`), so the multi-second load/warm never freezes the GUI. The
  worker (`loadWindowJob`) must touch NO Qt/GUI state — it returns a `LoadResult`
  (a `shared_ptr<ReSimEngine>` + the loaded `shared_ptr<LoadedBag>`) installed on
  the GUI thread in `onLoadFinished`. One load in flight at a time; a newer
  request supersedes via `request_id_` / `chase_target_s_`. A load is **two
  stages**: stage A reads the window + builds a *display-only* engine
  (`seekToStampDisplayOnly` — images, no warm-up); stage B builds a fresh engine
  from the **same bag** (cv::Mat is refcounted, so this copies frame headers, not
  pixels) and warms it. `~MainWindow` waits for any in-flight load.
- **Tuning is preserved across reloads via `applied_occ_`/`applied_acc_`**, the
  source of truth threaded into every (re)load — NOT read back off the engine.
  The engine ctor only takes `occ` + geometry, so all four tunable
  `AccumulateParams` knobs are applied after construction in `loadWindowJob`;
  forgetting this silently resets dock tuning on the next scrub-triggered reload.
- **`renderViews()` vs `rescaleViews()`** — rendering the costmaps is a per-pixel
  `n²` cell loop, far too slow to run on every resize / `splitterMoved` tick (it
  was, and the UI was sluggish). `renderViews()` re-renders into cached `QImage`s
  only when engine state changes (seek/load/apply); `rescaleViews()` just
  `QPixmap::scaled` the cache and runs on resize/splitter drag. Keep engine/render
  calls out of `rescaleViews()`.
- **Costmap render spans the buffer window, not `panel_px·res`** — `renderGrid`/
  `renderRecorded` map panel pixels over `[−half_extent, +half_extent]`
  (`mpp = 2·half_extent/panel_px`), so the panel always covers the buffer window
  regardless of `panel_px` or `--res`/`--window-m`. (An earlier version stepped by
  `acc_.res`, which only matched the window by the coincidence
  `panel_px(480)·res(0.25)=120m` at defaults.)
- **The BufferPlan Extend optimization is NOT wired to the loader yet.**
  `plan_buffer` computes `action`/`read_lo`/`read_hi` for an incremental extend
  (and `test_buffer_policy` asserts it), but `loadWindowJob` always
  `loadWindow(cache_lo, cache_hi)` — a full re-read of the guaranteed window. It's
  correct (superset), just not minimal I/O, and `loadWindow` still linearly skips
  to the window rather than `Reader::seek`. The remaining perf work is: splice an
  extension onto a retained buffer + seek. Don't read the passing extend tests as
  proof the loader does incremental reads.
