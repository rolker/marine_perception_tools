# Plan: sea_surface_tuner — interactive C++/Qt bag-replay + costmap tuner

## Issue

https://github.com/rolker/marine_perception_tools/issues/1

## Context

The repo skeleton exists on `jazzy`: an `ament_cmake` package building a
placeholder `sea_surface_tuner` (empty `QMainWindow`), Qt5 with
version-agnostic CMake. This plan adds the `sea_surface_segmentation`
dependency and the actual MVP tuner on top.

The real algorithm is reused via the exported headers from
`unh_marine_perception#23` — no reimplementation:
- `occupancy_buffer.hpp` — `OccupancyBuffer` + `OccupancyParams`
  (`hit_log_odds`, `miss_log_odds`, `clamp`, `lethal_threshold`,
  `decay_half_life_s`); `setParams`/`validate`/`clear`/`logOdds`.
- `occupancy_accumulator.hpp` — `accumulate_frame()` + `AccumulateParams`
  (`max_range`, `res`, `half_extent`, `plane_z`, `min_grazing_angle_deg`).
- `segments_projection.hpp` — `project_observations_inverse`.
- `tools/bag_to_costmap_video.cpp` — the existing offline driver; its
  two-pass bag read (pass 1: TF buffer + camera models; pass 2: replay) and
  its `render_new()` (log-odds → BGR panel) are the template to extract.

**Open technical items from the issue — resolved by reading the driver:**
- The segmentation that drives the costmap is a **raw `rgb8` `Image`** on
  `/bizzy/sensors/cameras/<cam>/segmentation`, decoded with `cv_bridge` — it is
  **in the main bag, not ffmpeg-encoded.** (`occupancy_accumulator.hpp`
  comment: "the ffmpeg-encoded stream is the display camera imagery, never
  projected.") So the MVP needs no ffmpeg decode.
- `camera_info` is on `.../segmentation/camera_info`; TF on `/tf`+`/tf_static`;
  world frame `bizzy/map_tide`, boat `bizzy/base_link`; forward camera
  `oak_forward`. All present in the bag and consumed by the existing driver.

## Approach

Mirror the live layer's praised separation: a Qt-free, ROS-free **re-sim core**
+ a thin ROS **bag loader** + a Qt **UI**.

1. **Add dependencies** — `package.xml`/`CMakeLists.txt`: `sea_surface_segmentation`
   (exported headers), `grid_map_core`, `image_geometry`, `cv_bridge`,
   `rosbag2_cpp`, `sensor_msgs`, `nav_msgs`, `tf2`, `tf2_msgs`,
   `builtin_interfaces`, `libopencv-dev`. Keep Qt5 wiring as-is.

2. **`bag_loader` (ROS, no Qt)** — `src/bag_loader.{hpp,cpp}`. Extract the
   driver's two passes for the **forward camera only**: pass 1 fills a
   `tf2::BufferCore` + the `oak_forward` `PinholeCameraModel`; pass 2 decodes
   each `oak_forward/segmentation` frame to `rgb8` `cv::Mat`, resolves
   `camera_origin` / `rotation_cam_to_target` / `boat_x,y` from TF at the frame
   stamp, and produces an in-memory `std::vector<PreparedFrame>` over the
   requested `[start_s, end_s]` window (the issue's "frame window preloaded in
   memory"). `PreparedFrame{ stamp_s, cv::Mat mask_rgb8, cv::Vec3d origin,
   cv::Matx33d rot, double boat_x, boat_y }`. No occupancy logic here.

3. **`resim_engine` (pure: sea_surface_segmentation + OpenCV + grid_map, no Qt/no
   ROS-msgs)** — `src/resim_engine.{hpp,cpp}`. Owns an `OccupancyBuffer`,
   `OccupancyParams`, `AccumulateParams`. Given the prepared frames and a target
   index `k`, **`resimulateTo(k)`** does `clear()` then replays frames `0..k`
   through `accumulate_frame()` — path-dependent, exactly the live accumulation
   order. `renderGrid(...)` reproduces `render_new()` (boat-centred, N-up,
   log-odds palette) into a `cv::Mat`. `setOccupancyParams`/`setAccumulateParams`
   call `OccupancyBuffer::validate()` first and re-sim on the current index.

4. **Qt UI** — extend `MainWindow`. Two image panes side-by-side
   (`oak_forward` segmentation `rgb8` | re-sim lethal grid) via a `cv::Mat`→
   `QImage` helper (`src/cv_qt.hpp`). A `QSlider` scrubber over frame index
   (drives `resimulateTo`). A parameter dock: one `QDoubleSpinBox` per knob
   (5 occupancy + `max_range`, `min_grazing_angle_deg`); editing re-sims and
   refreshes the grid pane. Invalid params (rejected by `validate()`) surface a
   status-bar message and revert the spinbox.

5. **CLI entry** — `main.cpp` parses `<bag_uri> [--start-s] [--end-s] [--res]
   [--window-m] [--max-range]` (mirror the driver's flags), loads frames, hands
   them to the engine + window. App usage documented in README.

6. **Tests** — `test/test_resim_engine.cpp` (GTest), no bag/Qt: feed synthetic
   `PreparedFrame`s (tiny hand-built `rgb8` masks + identity-ish geometry) and
   assert (a) determinism: same params+frames → identical buffer; (b) re-sim
   equivalence: `resimulateTo(k)` matches a direct fresh `accumulate_frame` loop;
   (c) a param change followed by re-sim equals constructing the buffer fresh
   with the new params (no residue from the prior run — guards the `clear()`);
   (d) `validate()` rejection leaves the engine state unchanged.

## Files to Change

| File | Change |
|------|--------|
| `package.xml` | Add the deps in step 1 (+ `ament_cmake_gtest` test dep) |
| `CMakeLists.txt` | `find_package` the deps; build `bag_loader`+`resim_engine` into the exe; `ament_add_gtest` for the engine test |
| `src/bag_loader.hpp` / `.cpp` | New — two-pass forward-camera bag → `vector<PreparedFrame>` |
| `src/resim_engine.hpp` / `.cpp` | New — `OccupancyBuffer` wrapper, `resimulateTo`, `renderGrid`, param setters |
| `src/cv_qt.hpp` | New — `cv::Mat`(BGR/rgb8) → `QImage`/`QPixmap` helper |
| `src/main_window.hpp` / `.cpp` | Two image panes, scrubber, param dock, status-bar validation feedback |
| `src/main.cpp` | CLI arg parse → load frames → construct engine + window |
| `test/test_resim_engine.cpp` | New — GTest for the re-sim core |
| `README.md` | Replace "skeleton" status with MVP usage + screenshot-less feature list |
| `.agents/README.md` | Update package inventory/layout to the real modules |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | MVP = forward camera, the 7 listed knobs, seg+grid panes, scrubber, re-sim. 4-camera mosaic, recorded-costmap overlay, #22 gate knobs, param export → deferred to "Full" (issue already scopes this). |
| Test what breaks | The path-dependent re-sim + param-change `clear()` is the breakable logic; covered by the engine GTest. UI is thin and excluded (no display in CI). |
| A change includes its consequences | Adds the `sea_surface_segmentation` cross-layer dep (ui→sensors, valid); README + `.agents/README.md` updated in the same PR; CI already builds+lints. |
| Capture decisions | Qt5/version-agnostic rationale already in CMake + README; any non-obvious UI pivot → `## Implementation Notes`. |
| Primary framework first, portability where free | Qt5 (Jazzy) now; version-agnostic CMake keeps the Qt6 path free. Reuse the real algorithm rather than fork it — offline result matches the boat. |

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

## Open Questions

- **Bag availability for manual verification.** The engine test is synthetic; a
  real `#186` bag with `oak_forward/segmentation` + `camera_info` + TF over
  `bizzy/map_tide`→`base_link` is needed to eyeball the tuner. Confirm a usable
  bag path (e.g. under `~/data/logs/bizzy*`) before the verify step. Does not
  block implementation.
- **Stacked vs single PR.** This is implementable as one PR, or split: PR-A =
  `bag_loader` + `resim_engine` + a headless `--dump-frame N out.png` CLI
  (fully testable, proves the #23 reuse); PR-B = the Qt UI on top. Recommend
  **single PR** unless review prefers the split — flagging per
  "surface scope deferrals."

## Estimated Scope

Single PR (~500–700 lines incl. test). Cleanly splittable into a headless-core
PR + a Qt-UI PR if preferred.
