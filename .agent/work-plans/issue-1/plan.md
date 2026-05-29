# Plan: sea_surface_tuner — interactive C++/Qt bag-replay + costmap tuner

## Issue

https://github.com/rolker/marine_perception_tools/issues/1

## Context

The repo skeleton exists on `jazzy`: an `ament_cmake` package building a
placeholder `sea_surface_tuner` (empty `QMainWindow`), Qt5 with
version-agnostic CMake. This plan adds the `sea_surface_segmentation`
dependency and the actual MVP tuner on top.

The **algorithm** is reused via the exported headers from
`unh_marine_perception#23` — no reimplementation of the marking logic:
- `occupancy_buffer.hpp` — `OccupancyBuffer` + `OccupancyParams`
  (`hit_log_odds`, `miss_log_odds`, `clamp`, `lethal_threshold`,
  `decay_half_life_s`); `setParams`/`validate`/`clear`/`logOdds`. Note
  `validate()` checks **only `OccupancyParams`** — the `AccumulateParams`
  knobs below have no library validator (the driver does its own `>0` checks).
- `occupancy_accumulator.hpp` — `accumulate_frame()` + `AccumulateParams`
  (`max_range`, `res`, `half_extent`, `plane_z`, `min_grazing_angle_deg`).
  `res`/`half_extent` define the iteration **window geometry**, which is fixed
  at `OccupancyBuffer` construction (`setGeometry`) — they are *not*
  runtime-`setParams`-able (see step 3).
- `segments_projection.hpp` — `project_observations_inverse`.
- `tools/bag_to_costmap_video.cpp` — the existing offline driver; its
  two-pass bag read (pass 1: TF buffer + camera models; pass 2: replay) is the
  template to extract. Its `render_new()`/`colour_logodds()` are in the tool's
  **anonymous namespace (not exported)**, so the grid rendering is *reproduced*
  (~15 lines copied), not linked — a cosmetic palette that can drift from the
  tool independently of the (reused) algorithm.

**Open technical items from the issue — resolved by reading the driver:**
- The segmentation that drives the costmap is a **raw `rgb8` `Image`** on
  `/bizzy/sensors/cameras/<cam>/segmentation`, decoded with `cv_bridge` — it is
  **in the main bag, not ffmpeg-encoded.** (`occupancy_accumulator.hpp`
  comment: "the ffmpeg-encoded stream is the display camera imagery, never
  projected.") So the MVP needs no ffmpeg decode.
- `camera_info` is on `.../segmentation/camera_info`; TF on `/tf`+`/tf_static`;
  world frame `bizzy/map_tide`, boat `bizzy/base_link`; forward camera
  `oak_forward`. All present in the bag and consumed by the existing driver.

**Coupling to `unh_marine_perception#22` (decided sequencing).** #22 (plan PR
#25, not yet implemented) rewrites the marking algorithm and its *parameter
model*: it drops the argmax gate for per-pixel softmax log-odds
(`Δ=clamp(log(R/(255−R)))`), so the fixed `hit_log_odds`/`miss_log_odds`
increments stop being the vote source; splits the symmetric `clamp` into
`obstacle_ceiling`/`clear_floor`; adds `reflex_tau`; and (P2) adds a graded cost
ramp (`clear_thresh`/`lethal_thresh`). The tuner is #22's validation harness, so
its full param dock must target #22's model — but #22 isn't built yet.
**Decision: build the algorithm-agnostic pipeline now; the param dock for the
#22-specific knobs lands after #22-P1.** The split below reflects this. The
pipeline (`bag_loader`, `resim_engine`, UI shell, scrubber, panes) reuses
`accumulate_frame`/`render_new` whose *signatures* are stable across #22 (only
the param structs + internals change), so none of it is throwaway.

## Approach

Mirror the live layer's praised separation: a Qt-free, ROS-free **re-sim core**
+ a thin ROS **bag loader** + a Qt **UI**. Delivered as two stacked PRs driven by
the #22 coupling above:

- **Milestone A (PR-A, now):** pipeline + replay/re-sim viewer + a param dock for
  only the **#22-stable knobs** (`decay_half_life_s`, `lethal_threshold`,
  `max_range`, `min_grazing_angle_deg`, window `res`/`half_extent`), with re-sim
  on change. A fully functional bag-replay costmap viewer.
- **Milestone B (PR-B, after #22 — now merged, PR #25):** extend the
  (data-driven) dock to #22's model. The merged `OccupancyParams` replaced
  `hit/miss/clamp` with **`obstacle_clamp`, `clear_floor`, `free_threshold`**
  (+ existing `lethal_threshold`, `decay_half_life_s`); `AccumulateParams` added
  **`obstacle_prob_min`, `max_evidence_step`**. (No `reflex_tau` in the costmap
  params — the reflex gate lives in `segments_to_pointcloud`, outside the
  buffer/accumulator the tuner drives.) Adding these rows makes it a #22 *tuning*
  harness.

1. **Add dependencies** — `package.xml`/`CMakeLists.txt`: `sea_surface_segmentation`
   (exported headers), `grid_map_core`, `image_geometry`, `cv_bridge`, `rclcpp`,
   `rosbag2_cpp`, `sensor_msgs`, `tf2`, `tf2_msgs`, `builtin_interfaces`,
   `libopencv-dev`. (`nav_msgs` is *not* needed in Milestone A — it's only for the
   recorded-costmap overlay, deferred to "Full".) Keep Qt5 wiring as-is.

2. **`bag_loader` (ROS, no Qt)** — `src/bag_loader.{hpp,cpp}`. Extract the
   driver's two passes for the **forward camera only**: pass 1 fills a
   `tf2::BufferCore` + the `oak_forward` `PinholeCameraModel`; pass 2 decodes
   each `oak_forward/segmentation` frame to `rgb8` `cv::Mat`, resolves
   `camera_origin` / `rotation_cam_to_target` / `boat_x,y` from TF at the frame
   stamp, and produces an in-memory `std::vector<PreparedFrame>` over the
   requested `[start_s, end_s]` window (the issue's "frame window preloaded in
   memory"). `PreparedFrame{ stamp_s, cv::Mat mask_rgb8, cv::Vec3d origin,
   cv::Matx33d rot, double boat_x, boat_y }`. No occupancy logic here.
   - **TF-lookup failures** (early frames before TF is populated, or a gap) are
     handled like the driver: **skip** that frame (don't abort the load, don't
     emit a `PreparedFrame` with stale geometry). Hard errors — no
     `oak_forward` segmentation/`camera_info` topic at all, or zero usable
     frames — fail loudly.
   - **`plane_z`**: the world frame is `bizzy/map_tide` (tide-corrected), so the
     water plane is `plane_z = 0` — the driver's convention; the loader sets it
     once, not per frame.

3. **`resim_engine` (pure: sea_surface_segmentation + OpenCV + grid_map, no Qt/no
   ROS-msgs)** — `src/resim_engine.{hpp,cpp}`. Owns an `OccupancyBuffer`,
   `OccupancyParams`, `AccumulateParams`. Constructed with the fixed **window
   geometry** (`res`, `half_extent`) — these size the `OccupancyBuffer` and are
   **not** live-tunable (changing them would require rebuilding the buffer), so
   they are set once from CLI args (see step 5), not from the dock. Given the
   prepared frames and a target index `k`, **`resimulateTo(k)`** does `clear()`
   then replays frames `0..k` through `accumulate_frame()` — path-dependent,
   reproducing the **#23 shared offline core** (`bag_to_costmap_video`)
   accumulation order. (Fidelity caveat: `accumulate_frame` iterates a
   **boat-centred** window; the *live* `SeaSurfaceLayer` centres iteration on the
   camera instead — so the tuner matches the offline core exactly, and the live
   layer up to that small window-centre offset.) `renderGrid(...)` reproduces
   `render_new()` (boat-centred, N-up, log-odds palette) into a `cv::Mat`.
   - `setOccupancyParams(p)` — runs `OccupancyBuffer::validate(p)` first; on pass,
     `setParams` + re-sim on the current index; on fail, returns the reason
     (no state change).
   - `setProjectionParams(max_range, min_grazing_angle_deg)` — the live-tunable
     `AccumulateParams` knobs (projection only, no buffer rebuild). The library
     has **no validator** for these, so the engine bounds-checks them itself
     (`max_range > 0`; `0 ≤ min_grazing_angle_deg < 90`), mirroring the driver,
     then re-sims. Geometry (`res`/`half_extent`) is intentionally not settable.

4. **Qt UI** — extend `MainWindow`. Two image panes side-by-side
   (`oak_forward` segmentation `rgb8` | re-sim lethal grid) via a `cv::Mat`→
   `QImage` helper (`src/cv_qt.hpp`). A `QSlider` scrubber over frame index
   (drives `resimulateTo`). **Data-driven param dock**: a table of
   `{label, getter, setter, range}` rows renders one `QDoubleSpinBox` each;
   editing re-sims and refreshes the grid pane; invalid params (rejected by
   `validate()`) surface a status-bar message and revert the spinbox. The
   table-driven design is deliberate — Milestone B swaps the knob set for #22's
   model by editing the table, not the widget code.
   - **Milestone A dock knobs (#22-stable, live-tunable only):**
     `decay_half_life_s`, `lethal_threshold` (`OccupancyParams`, validator-backed)
     and `max_range`, `min_grazing_angle_deg` (`AccumulateParams`, engine
     bounds-checked). Window `res`/`half_extent` are **CLI-only** (construction
     geometry, not live-tunable). (`hit_log_odds`/`miss_log_odds`/`clamp` are
     intentionally *omitted* now — #22-P1 removes/replaces them; exposing them
     would tune soon-dead knobs.)
   - **Milestone B (after #22, merged):** add `obstacle_clamp`, `clear_floor`,
     `free_threshold` (`OccupancyParams`) and `obstacle_prob_min`,
     `max_evidence_step` (`AccumulateParams`) as table rows.

5. **CLI entry** — `main.cpp` parses `<bag_uri> [--start-s] [--end-s] [--res]
   [--window-m] [--max-range]` (mirror the driver's flags), loads frames, hands
   them to the engine + window. App usage documented in README.

6. **Tests** — `test/test_resim_engine.cpp` (GTest), no bag/Qt: feed synthetic
   `PreparedFrame`s (tiny hand-built `rgb8` masks + identity-ish geometry) and
   assert (a) determinism: same params+frames → identical buffer; (b) re-sim
   equivalence: `resimulateTo(k)` matches a direct fresh `accumulate_frame` loop;
   (c) a param change followed by re-sim equals constructing the buffer fresh
   with the new params (no residue from the prior run — guards the `clear()`);
   (d) rejection paths leave engine state unchanged — both `setOccupancyParams`
   with a `validate()`-failing value and `setProjectionParams` with an
   out-of-bounds `max_range`/`min_grazing_angle_deg`.

## Files to Change

| File | Change |
|------|--------|
| `package.xml` | Add the deps in step 1 (+ `ament_cmake_gtest` test dep) |
| `CMakeLists.txt` | `find_package` the deps; build `bag_loader`+`resim_engine` into the exe; `ament_add_gtest` for the engine test |
| `src/bag_loader.hpp` / `.cpp` | New — two-pass forward-camera bag → `vector<PreparedFrame>` |
| `src/resim_engine.hpp` / `.cpp` | New — `OccupancyBuffer` wrapper, `resimulateTo`, `renderGrid`, param setters |
| `src/cv_qt.hpp` | New — `cv::Mat`(BGR/rgb8) → `QImage`/`QPixmap` helper |
| `src/main_window.hpp` / `.cpp` | Two image panes, scrubber, **data-driven** param dock (Milestone-A knobs), status-bar validation feedback |
| `src/main.cpp` | CLI arg parse → load frames → construct engine + window |
| `test/test_resim_engine.cpp` | New — GTest for the re-sim core |
| `README.md` | Replace "skeleton" status with viewer usage + feature list |
| `.agents/README.md` | Update package inventory/layout to the real modules |

(Milestone B, after #22-P1, edits the dock's knob table + `README` only.)

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Milestone A = forward camera, seg+grid panes, scrubber, re-sim, dock for #22-*stable* knobs only. 4-camera mosaic, recorded-costmap overlay, param export → "Full". The #22 gate/grading knobs deferred to Milestone B (after #22-P1) so we don't build a dock for params #22 is removing. |
| Test what breaks | The path-dependent re-sim + param-change `clear()` is the breakable logic; covered by the engine GTest. UI is thin and excluded (no display in CI). |
| A change includes its consequences | Adds the `sea_surface_segmentation` cross-layer dep (ui→sensors, valid); README + `.agents/README.md` updated in the same PR; CI already builds+lints. |
| Capture decisions | Qt5/version-agnostic rationale already in CMake + README; any non-obvious UI pivot → `## Implementation Notes`. |
| Primary framework first, portability where free | Qt5 (Jazzy) now; version-agnostic CMake keeps the Qt6 path free. Reuse the real algorithm rather than fork it — the re-sim reproduces the #23 offline core exactly (and the live layer up to the boat-vs-camera window-centre offset noted in step 3). |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 (ROS 2 conventions) | Yes | `ament_cmake`, standard package layout, `ament_add_gtest`, rosbag2/cv_bridge/image_geometry idioms mirrored from the existing driver |
| ADR-0004/0005 (enforcement) | Yes | Pre-commit + CI already in the skeleton; engine test adds the functional gate |
| ADR-0002 (worktree isolation) | Yes | Work is in `feature/issue-1` worktree on `marine_perception_tools` |
| ADR-0009 (Python pkg policy) | No | No Python in MVP |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `package.xml` deps | `CMakeLists.txt` find_package + CI rosdep resolves them | Yes |
| Reuse `accumulate_frame`/`render_new` | Stay in lockstep with `sea_surface_segmentation` exported API; if #23 headers change signature, the tuner breaks at build | Yes — pinned to jazzy; CI catches |
| README "skeleton" status | `.agents/README.md` package inventory | Yes |
| New `oak_forward` assumptions | If a bag lacks `oak_forward` seg/camera_info/TF, loader must fail loudly (not silently empty) | Yes — explicit error |
| #22-P1 lands (param model changes) | Dock knob table (Milestone B), README; `accumulate_frame`/`render_new` callers pick up new behavior automatically | Milestone B — table-driven dock makes it a small edit |

## Open Questions

- **Bag availability for manual verification.** The engine test is synthetic; a
  real `#186` bag with `oak_forward/segmentation` + `camera_info` + TF over
  `bizzy/map_tide`→`base_link` is needed to eyeball the tuner. Confirm a usable
  bag path (e.g. under `~/data/logs/bizzy*`) before the verify step. Does not
  block implementation.
- ~~Stacked vs single PR~~ **Resolved (user, 2026-05-29):** stacked, driven by
  the #22 coupling — **Milestone A (PR-A)** = pipeline + viewer + #22-stable dock,
  buildable now; **Milestone B (PR-B)** = #22 knob set, after #22-P1 is
  implemented.
- **Milestone B depends on `unh_marine_perception#22` Phase 1** (plan PR #25)
  being implemented and the new `OccupancyParams`/`AccumulateParams` fields
  landing on `jazzy`. Track #25; PR-B starts when those headers update.

## Estimated Scope

Two stacked PRs. **PR-A** (now) ~450–600 lines incl. test. **PR-B** (after
#22-P1) small — a dock knob-table edit + README.

## Implementation Notes

- **#22 sequencing (2026-05-29).** `unh_marine_perception#22` (plan PR #25) was
  found mid-planning to replace the tuner's parameter model (per-pixel softmax
  log-odds, asymmetric clamp, reflex gate, graded ramp). Rather than build a
  dock for the soon-removed `hit/miss/clamp` knobs, the work was split: the
  algorithm-agnostic pipeline + viewer + a dock for #22-stable knobs lands now
  (Milestone A); the #22 knob set lands after #22-P1 (Milestone B). The
  param dock is built table-driven specifically so Milestone B is a data edit,
  not a UI rewrite. Decision confirmed by Roland.
- **Window geometry is CLI-only (plan-review finding 1).** `res`/`half_extent`
  size the `OccupancyBuffer` at construction (`setGeometry`); there's no resize,
  so making them live dock knobs would force a buffer rebuild on every edit.
  They're set once from CLI args instead. The live dock holds only genuinely
  `setParams`-able / projection knobs. `OccupancyBuffer::validate()` covers only
  `OccupancyParams`, so the engine bounds-checks the `AccumulateParams` knobs
  (`max_range`, `min_grazing_angle_deg`) itself.
