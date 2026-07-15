# Plan: Survey explorer stage 3: multi-pass MBES pointcloud drill-down

## Issue

https://github.com/rolker/marine_perception_tools/issues/21

## Context

Stage 2 (PR #20) landed `SurveyOverviewWindow`: click the bathymetry map, see
which passes ensonified that spot, double-click a pass to open `SidescanViewerWindow`
for a single bag. Stage 3 adds the MBES drill-down: select multiple passes from the
result list and display their soundings together in one 3D point cloud, each pass
colour-coded by index.

Current state:
- `SurveyOverviewWindow` has a single-select `QTreeWidget` (`pass_list_`); activating
  an item opens a `SidescanViewerWindow` (sidescan scrub viewer, not the point cloud).
- `PointCloudView` renders one flat `std::vector<MbesSounding>` coloured by depth or
  backscatter; it knows nothing about multi-pass.
- `SidescanBagSession` builds a full bag distance index before any windowed read —
  appropriate for the scrub viewer, expensive for a one-shot MBES pull from a known
  time window.
- No class currently reads MBES soundings from a bag for a known `[t_start_ns,
  t_end_ns]` window without a full index pass.

## Approach

1. **Add `MbesBagReader` — lightweight windowed MBES read** — new
   `src/mbes_bag_reader.hpp` + `src/mbes_bag_reader.cpp`. Given a bag path and a
   `[t_start_ns, t_end_ns]` window:
   - Opens the bag (rosbag2 Reader).
   - Linear-scans, collecting TF records first (one pass, sized `BufferCore`) then
     MBES detections in the window. Window is bounded, so the scan covers only the
     pass duration (seconds to minutes), not the full bag.
   - Projects each detection's beams to world-frame `MbesSounding` via TF2
     (consistent with `SidescanBagSession` — ADR-0008 compliance). Skips pings with
     no TF.
   - Returns `std::vector<MbesSounding>` (world frame) + skipped-ping count.
   - Stateless free function `read_mbes_window(bag_path, t_start_ns, t_end_ns,
     world_frame, m3_frame)` — no class state, easier to test.
   - **Design decision (windowed-read strategy)**: linear skip to `t_start_ns`
     then read until `t_end_ns`. `Reader::seek()` would be faster on long bags but
     is already noted as a future optimisation in `bag_loader.hpp`; the linear scan
     is proven and keeps the reader simple. Re-evaluate if load latency is
     unacceptable in practice (a 10-minute survey pass at ~5 Hz is ~3 000 pings).

2. **Extend `PointCloudView` for multi-pass colour** — add `ColorMode::Pass` and a
   new method `setMultiPassPoints(const std::vector<std::vector<MbesSounding>> &
   passes)` to `src/point_cloud_view.hpp/.cpp`.
   - Each pass index maps to a distinct hue via a golden-angle step
     `hue = (index * 137.5f) mod 360` (maximises perceptual separation up to ~10
     passes). Constant saturation (0.85) and value (0.9) keeps colours vivid.
   - Internally flattens to the existing `pts_`/`colors_` layout; `mode_` set to
     `Pass` bypasses the depth/backscatter ramp in `rebuild_colors()`.
   - The existing single-pass path (`setPoints`) is unchanged — `SidescanViewerWindow`
     does not need updating.
   - **Design decision (colour palette)**: fixed golden-angle HSV hues, not the
     marine_colormap depth ramp. Avoids conflating pass identity with depth. Dynamic
     palette (from marine_colormap categorical entries) is a follow-up if N > 10 is
     needed in practice.

3. **Add `MbesCloudWindow` — the multi-pass 3D viewer** — new
   `src/mbes_cloud_window.hpp` + `src/mbes_cloud_window.cpp`.
   - A `QMainWindow` containing a `PointCloudView` (left, large) and a legend panel
     (right) listing pass label, bag name, time range, ping count, and its colour
     swatch.
   - Constructor accepts `std::vector<PassInfo>` (bag_path + t_start_ns + t_end_ns +
     label). On show, kicks off a `QFuture<std::vector<std::vector<MbesSounding>>>`
     that calls `read_mbes_window` for each pass on a worker thread; `QFutureWatcher`
     marshals the result to the UI thread, then calls `setMultiPassPoints`.
   - Status bar shows loading progress (`"Loading pass N of M…"`) and the total
     sounding count on completion.
   - Each window is self-contained (`WA_DeleteOnClose`); the overview stays open as
     the navigation hub.

4. **Update `SurveyOverviewWindow`** — enable multi-pass selection and wire up
   the cloud action.
   - Change `pass_list_` from single-select (`QAbstractItemView::SingleSelection`)
     to extended-select (`QAbstractItemView::ExtendedSelection`).
   - Add a "View MBES cloud" `QPushButton` below the pass list (or a toolbar action).
     Enabled only when ≥ 1 pass item is selected.
   - `onViewMbesCloud()`: collect selected leaf items (skip bag header rows), build
     a `std::vector<PassInfo>`, create an `MbesCloudWindow`, call `show()`.
   - Keep `itemActivated` for double-click → `SidescanViewerWindow` (single-pass
     sidescan scrub, unchanged).

5. **Tests**
   - `test/test_mbes_bag_reader.cpp` (NEW): write a minimal synthetic bag with a
     known MBES detection and a TF record; assert `read_mbes_window` returns the
     expected sounding count and that world-frame coordinates are within tolerance.
     Test the zero-detection and TF-miss cases.
   - `test/test_point_cloud_view.cpp` (EXTEND): add a `MultiPassColour` test that
     calls `setMultiPassPoints` with two passes of different colours and asserts
     rendered pixels are non-background (mirrors the existing `RendersAndClears`
     pattern); add a `ColorModePass` setter safety test.

6. **CMakeLists.txt** — add `src/mbes_bag_reader.cpp` and
   `src/mbes_cloud_window.cpp` to `sidescan_target_viewer` sources; add
   `test_mbes_bag_reader` test target linked against `rosbag2_cpp` and
   `marine_acoustic_msgs`.

7. **README update** — add a "Survey explorer stage 3" section under the existing
   `sidescan_target_viewer` notes describing the multi-pass MBES cloud feature.

## Files to Change

| File | Change |
|------|--------|
| `src/mbes_bag_reader.hpp` | NEW — `read_mbes_window` declaration |
| `src/mbes_bag_reader.cpp` | NEW — windowed MBES bag read + TF2 projection |
| `src/mbes_cloud_window.hpp` | NEW — `MbesCloudWindow` declaration |
| `src/mbes_cloud_window.cpp` | NEW — multi-pass 3D viewer window |
| `src/point_cloud_view.hpp` | Add `ColorMode::Pass`, `setMultiPassPoints()` |
| `src/point_cloud_view.cpp` | Implement `ColorMode::Pass` in `rebuild_colors()`, `setMultiPassPoints()` |
| `src/survey_overview_window.hpp` | Add `onViewMbesCloud()` slot |
| `src/survey_overview_window.cpp` | Extended-select, "View MBES cloud" button, `onViewMbesCloud` |
| `test/test_mbes_bag_reader.cpp` | NEW — windowed reader unit test |
| `test/test_point_cloud_view.cpp` | Extend with `MultiPassColour` + `ColorModePass` tests |
| `CMakeLists.txt` | Add new sources + `test_mbes_bag_reader` target |
| `README.md` | Stage 3 section under `sidescan_target_viewer` |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Multi-select + explicit button action; no hidden automation. Pass colour legend makes per-pass identity visible. |
| Only what's needed | `MbesBagReader` is minimal — a free function, not a class with state. `MbesCloudWindow` doesn't reproduce the full scrub viewer. |
| A change includes its consequences | `PointCloudView` interface extension tested in `test_point_cloud_view`; single-pass callers (`SidescanViewerWindow`) verified unchanged. README updated same PR. |
| Test what breaks | Tests target the windowed-read path (the new regression risk) and the multi-pass colour mode, not the framework glue. |
| Improve incrementally | Stage 3 adds the multi-pass viewer; CUBE generation, sidescan draping, and contacts overlay remain explicitly out of scope. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (Adopt ADRs) | Watch | Two design decisions (windowed-read strategy, colour palette) documented inline in this plan. No separate ADR needed — they are implementation choices, not workspace-governance decisions. |
| ADR-0002 (Worktree isolation) | Yes | Working on `feature/issue-21` in `issue-marine_perception_tools-21` worktree. |
| ADR-0008 (ROS 2 conventions) | Watch | `MbesBagReader` uses `tf2::BufferCore` for georeferencing, consistent with `SidescanBagSession` (same TF2 path the indexer uses). ROS Rolling conventions observed for bag reader API. |
| ADR-0013 (progress.md vocabulary) | Yes | Plan Authored entry written to `.agent/work-plans/issue-21/progress.md`. |

## Consequences

| If we change… | Also update… | Included in plan? |
|---|---|---|
| `point_cloud_view.hpp` (new method/enum) | `SidescanViewerWindow` callers of `setPoints` — verify no breakage | Yes — single-pass path unchanged; tested |
| Add `mbes_bag_reader.cpp` | `CMakeLists.txt` source list + test target | Yes — step 6 |
| `SurveyOverviewWindow` (extended-select) | Existing `itemActivated` double-click path — must still open sidescan viewer | Yes — kept unchanged in step 4 |
| README stage 3 section | None — no cross-repo doc references for stage 3 yet | Yes — step 7 |

## Open Questions

- None — plan is review-plan-ready.

## Estimated Scope

Single PR. Approximately 8 files new/modified; the most complex new piece is
`mbes_bag_reader.cpp` (windowed bag read + TF2). All changes are in
`marine_perception_tools`.
