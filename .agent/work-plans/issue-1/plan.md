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

- **Milestone A:** pipeline + replay/re-sim viewer + frame scrubber.
- **Milestone B — folded into the same PR** once #22 (PR #25) merged
  mid-implementation (the A/B split's rationale — "don't build a dock for
  soon-changing knobs" — was gone). The dock exposes the full live-tunable #22
  set: `OccupancyParams` `decay_half_life_s`, `lethal_threshold`,
  `obstacle_clamp`, `clear_floor`, `free_threshold`; `AccumulateParams`
  `max_range`, `min_grazing_angle_deg`, `obstacle_prob_min`, `max_evidence_step`.
  (No `reflex_tau` in the costmap params — the reflex gate lives in
  `segments_to_pointcloud`, outside the buffer/accumulator the tuner drives.)
  Window geometry (`res`/`half_extent`) stays CLI-only.

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
   - `setAccumulateParams(p)` — the live-tunable `AccumulateParams` knobs
     (`max_range`, `min_grazing_angle_deg`, `obstacle_prob_min`,
     `max_evidence_step`; no buffer rebuild). The library has **no validator** for
     these, so the engine bounds-checks them itself (`max_range > 0`;
     `0 ≤ min_grazing_angle_deg < 90`; `obstacle_prob_min ∈ [0,1]`;
     `max_evidence_step > 0`), then re-sims. The window-geometry fields
     (`res`/`half_extent`/`plane_z`) of `p` are ignored — construction-fixed.

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
- **Milestone B folded into PR #2 (2026-05-29).** #22 (PR #25) merged while
  Milestone A was in review, so rather than ship the stable-knob subset and a
  follow-up PR, the full #22 knob set was added to the dock in the same PR
  (Roland's call — "fold into PR #2, all 5 new tunables"). The engine grew a
  `setAccumulateParams` (mirroring `setOccupancyParams`) so the `AccumulateParams`
  knobs are tunable too; window geometry stays construction-fixed. The
  table-driven dock made this a member-pointer table edit, as designed.

## Milestone C — interactive 4-camera tuner (2026-05-29)

Folded into PR #2 at Roland's direction after the deployment window was missed,
so the tool can be finished deliberately rather than fast. Three asks (confirmed
via questions): **(1) menu bar with File→Open, (2) all four cameras, (3)
compressed images from the bag.** The requested layout:

```
Row 1  [ port/left RGB ] [ fwd RGB ] [ stbd/right RGB ] [ aft RGB ]   ← H.265 decode
Row 2  [ port seg      ] [ fwd seg ] [ stbd seg       ] [ aft seg ]   ← CompressedImage / Image
Row 3  [ recorded costmap (bag) ]            [ regenerated costmap (tuned) ]
       ◄──────────────── scrubber ────────────────►        Parameters dock ▸
```

### Ground truth (verified against real bags + deployed layer)

- **Cameras** (from `bag_to_costmap_video.cpp` `kCameras`, deployed convention):
  `oak_forward`, `oak_port`, `oak_starboard`, `oak_aft`. Display order
  left→right is **port, forward, starboard, aft** (= left, fwd, right, aft).
- **Bag source**: the `*_ffmpeg_seg` bags under `~/data/logs/bizzy_images/` are
  **self-contained** — all 4 cameras' `image_raw/ffmpeg` (`FFMPEGPacket`, H.265),
  `segmentation` (`Image`) **and** `segmentation/compressed` (`CompressedImage`)
  **and** `segmentation/camera_info`, plus `/tf` + `/tf_static` **and** the
  recorded `/bizzy/local_costmap/costmap` (`OccupancyGrid`). The main
  `bizzyboat/` nav bags have **none** of the camera/costmap topics. So File→Open
  targets one `*_ffmpeg_seg` bag; no second bag needed.
- **Fusion**: the deployed `SeaSurfaceLayer` fuses one-or-more cameras
  (`observation_sources`) into a **single** world-frame `OccupancyBuffer`. The
  driver does the same by interleaving all cameras' segmentation frames
  chronologically through one buffer (`accumulate_frame` re-centres + decays per
  call). So the tuner keeps **one** buffer and replays a **merged, time-sorted**
  segmentation stream across the 4 cameras — fusion is automatic.

### Design

- **`bag_loader`** generalises to the 4-camera set. `PreparedFrame` gains a
  `cam` index; `LoadedBag` holds a per-camera `camera_models` vector, the merged
  time-sorted segmentation `frames` (accumulation timeline), and two display-only
  side timelines: per-camera `rgb_frames` and recorded `costmaps`. Seg-source
  selection per camera: **prefer raw `Image`; fall back to `CompressedImage`**
  when the raw topic is absent (size-trimmed bags) — never read both for one
  camera (would double-count). Decoded via `cv_bridge::toCvCopy(..., "rgb8")`
  for either type. TF/CameraInfo handling unchanged (skip frames with no pose).
- **`resim_engine`** owns the `LoadedBag` by value (so the window can reload it).
  Accumulation timeline is the merged seg stream; `accumulate()` selects the
  per-frame camera model by `cam`. Adds display accessors: latest seg + latest
  RGB per camera ≤ current stamp, and the latest recorded costmap ≤ current
  stamp. `renderRecorded()` promotes the driver's `render_live()` (samples the
  `OccupancyGrid` into the same boat-centred window as `renderGrid`, shared
  palette) for the 1:1 comparison.
- **`main_window`** owns the engine and gains a **menu bar**: *File → Open Bag…*
  (`QFileDialog` → rebuild engine; tolerant of absent topics — missing RGB shows
  a placeholder), *File → Quit*. Launching with no bag arg starts empty. The
  pane grid becomes Rows 1–3 above; scrubber + table-driven param dock carry over
  unchanged (the dock still tunes the *fused* result).
- **H.265 RGB** (`image_raw/ffmpeg`, `FFMPEGPacket`) decoded with
  `ffmpeg_encoder_decoder::Decoder` (one per camera, fed in order, flushed) into
  `rgb_frames`. This is the heaviest, most isolable piece — Row 1 degrades to a
  placeholder if decode is unavailable, so the rest stands alone.

### New dependencies

`nav_msgs` (recorded `OccupancyGrid`), `ffmpeg_image_transport_msgs`
(`FFMPEGPacket`), `ffmpeg_encoder_decoder` (`Decoder`). All present on the Jazzy
operator station (`ros-jazzy-ffmpeg-image-transport*` installed).

### Open / verify items

- **Compressed segmentation encoding (PNG vs JPEG).** `segmentation/compressed`
  uses image_transport's `compressed` transport; if it is JPEG, lossy artefacts
  corrupt the R-channel obstacle probability the #22 softmax reads
  (`Δ=clamp(log(R/(255−R)))`). The tuner prefers the raw topic when present, so
  this only bites compressed-only bags — surface it (status-bar note when falling
  back to compressed), don't silently absorb. Verify the `format` field on a real
  `*_ffmpeg_seg` bag.
- **Fidelity caveat (unchanged)**: the re-sim matches the offline core exactly
  and the live layer up to the boat-vs-camera window-centre offset.

### Staging (atomic commits on `feature/issue-1`)

1. plan.md (this). 2. menu bar + File→Open + engine-owns-bag refactor.
3. CompressedImage seg support (prefer raw). 4. 4-camera loader + fused engine +
Rows 1–2. 5. recorded-costmap load/render + Row 3. 6. H.265 RGB decode + Row 1.
Tests + README + `.agents/README.md` grow alongside.

## Milestone D — windowed buffering for File→Open (2026-05-31)

**Problem (found in review of the built tool).** `MainWindow::openBag` calls
`load_bag(dir, load_opts_)` with `load_opts_` defaulting to the whole bag
(`end_s = -1`). So opening from the menu **buffers the entire recording** — every
segmentation frame plus every decoded H.265 RGB frame across all four cameras.
On a long 4-camera `*_ffmpeg_seg` bag that is a large, unbounded memory load.
Two separate issues: (1) File→Open inherits the CLI window (no menu-side window);
(2) even a CLI-windowed load full-scans the file twice and discards the TF cache.

**Decision (Roland, 2026-05-31, via questions).** On File→Open, buffer only what
the viewed moment needs, and let that buffer change as the timeline is scrubbed
**without re-opening the file**. The buffer is sized to the integration warm-up
the marking algorithm needs, plus a scrub margin on each side, plus a retention
budget so back-and-forth sweeps stay instant.

### The buffering model

For a view time `t` (the scrub point), define:

- **`integration` = `k × decay_half_life_s`**, `k = 3` (default). At the 30 s
  default half-life → 90 s. This is how far *back* the engine accumulates before
  `t`: `decay()` is exponential with no hard cutoff, so "fully warmed" is a
  multiple of the half-life (3× ⇒ ~12 % residual). Tying it to the knob means a
  change to `decay_half_life_s` in the dock recomputes the required window.
- **`margin` = 10 s** (default) — reload-free scrub slack on each side.
- **Guaranteed window** = `[t − integration − margin, t + margin]`, clamped to
  the bag's time bounds. Always resident. Re-sim still starts at
  `t − integration`; the extra `margin` before it just lets a small rewind avoid
  a reload.
- **Retention** = `retention_s` (default 120 s). Already-loaded frames beyond the
  guaranteed window are **kept**, not dropped, up to this budget. The loaded
  buffer is a single **contiguous** `[lo, hi]` span; scrubbing extends it; when
  its length exceeds `integration + 2·margin + retention_s` (≈230 s at defaults)
  the end **farthest in time from `t`** is trimmed — never inside the guaranteed
  window. So sweeping a region stays instant after the first pass.

### Scrub semantics

- The **scrubber spans the whole bag by time** (deciseconds; bounds from
  `reader.get_metadata()` at open — no frames loaded to populate it).
- Scrub to `t`:
  - **inside the loaded span** → map to frame index, `seekTo`, instant (in-memory).
  - **outside but overlapping** the guaranteed window → extend/reload the
    contiguous span to cover the new guaranteed window, trim per retention,
    rebuild the engine, seek.
  - **far jump (guaranteed window does not overlap the loaded span)** → **drop
    the current buffer and load fresh** around `t`. Non-overlapping frames can't
    contribute warm-up, and the engine needs one contiguous ascending timeline,
    so retention only helps contiguous scrubbing, not teleports (Roland confirmed).

### Architecture changes

- **`bag_loader` → stateful `BagSession`.** Split today's one-shot `load_bag`:
  - **`BagSession::open(uri)`** (once): the single full file scan — builds the
    persistent `tf2::BufferCore` (kept, not discarded), per-camera models,
    seg-source selection, and caches the bag time bounds + a topic/time index.
  - **`BagSession::loadWindow(start_s, end_s) → LoadedBag`**: `reader.seek()` +
    read to `end_ns`, projecting seg frames against the **already-built** TF cache
    and decoding RGB for just that span. No re-open, no re-scan.
  - The free `load_bag()` stays as a thin `open()+loadWindow()` wrapper so the
    synthetic-frame tests and `--probe` are untouched.
- **`ReSimEngine`: unchanged.** It already replays `[frame 0 … current]` from
  whatever `LoadedBag` it holds. If that bag *is* the window starting at
  `t − integration − margin`, replay-from-0 yields correct warm-up for free. The
  synthetic-frame unit tests keep passing as-is.
- **`MainWindow`: time-based scrubber + a buffer manager.** Holds the
  `BagSession`, the current loaded `[lo, hi]`, and the span policy. The span math
  — given `t`, integration, margin, retention, current span, and bag bounds,
  compute the next `[lo, hi]` and whether a reload/far-jump is needed — is
  extracted into a **pure free function** so it is unit-tested without a bag or
  Qt. Reloads are synchronous with a "Loading…" status (a ~110 s window loads in
  a few seconds; async is a later refinement if needed).

### New knobs

CLI + live-adjustable: `--integration-halflives` (default 3), `--margin-s`
(default 10), `--retention-s` (default 120). The existing `--start-s` / `--end-s`
remain as an optional hard clamp for power users (a windowed session inside an
explicit `[start,end]`).

### Tests (added to `test_resim_engine.cpp` or a new `test_buffer_policy.cpp`)

- Pure span-policy function: (a) open at `t=0` clamps `lo` to bag start (cold
  start, warms forward); (b) scrub inside loaded span ⇒ no reload; (c) scrub just
  outside ⇒ extend, guaranteed window covered; (d) retention trim drops the far
  end, never the guaranteed window; (e) far jump (no overlap) ⇒ full reload;
  (f) raising `decay_half_life_s` grows `integration` and forces the next reload.

### Principles / consequences

- **Only what's needed**: bounded memory is the whole point; the buffer is sized
  to the algorithm's warm-up, not the file.
- **Test what breaks**: the breakable logic is the span policy + the reuse of the
  cached TF across windows — both covered by the pure-function tests above; the
  full-bag `--probe` path stays as the loader integration check.
- **A change includes its consequences**: `README.md` (File→Open now windowed;
  document the new knobs + scrub behavior) and `.agents/README.md` (note the
  `BagSession` stateful loader) update in the same PR.
- **Capture decisions**: integration = 3× half-life, margin 10 s, retention
  120 s, far-jump = drop+reload — all recorded here with their rationale.
