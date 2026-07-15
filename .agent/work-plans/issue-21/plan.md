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

1. **Add windowed MBES reader to `sidescan_core`** — new
   `src/mbes_window_reader.hpp` + `src/mbes_window_reader.cpp`, built into the
   Qt-free `sidescan_core` library (the established home for bag ingest,
   alongside `sidescan_bag_session.cpp`). Stateless free function
   `read_mbes_window(bag_path, t_start_ns, t_end_ns, world_frame)` returning a
   result struct: soundings in the BAG's world frame, the captured
   earth<-world transform (see step 1b), and a skipped-ping count. Two-phase
   read, both phases topic-filtered (`rosbag2_storage::StorageFilter`):
   - **TF prepass**: read only `/tf` + `/tf_static` from the bag start until
     `t_end_ns + pad` into a `tf2::BufferCore` (must start from the beginning —
     static transforms and the chain history live there; TF messages are tiny
     so this is cheap). Also captures earth<-world for the window.
   - **Detections pass**: `Reader::seek(t_start_ns - pad)` with a try/catch
     sequential fallback — the same proven pattern as
     `SidescanBagSession::readMbesWindow` (`sidescan_bag_session.cpp:940`),
     which is the directly analogous precedent. For each detections message in
     the window: `project_detections()` (existing pure function in
     `mbes_geometry.hpp`) → lift to world via the shared TF helpers (step 1a).
     Pings with no resolvable TF are counted and skipped.
   - **Design decision (windowed-read strategy)**: seek + topic filters, NOT a
     full-bag linear scan and NOT a full `SidescanBagSession` per bag — the
     session's whole-bag metadata index (right for the scrub viewer, #17) is
     exactly the cost a one-shot windowed pull must avoid when several bags
     load at once.

1a. **Extract shared TF-lift helpers** — move `lookup_at_or_latest` and
   `rotate_by_quat` from `sidescan_bag_session.cpp`'s anonymous namespace into a
   new `src/tf_lift.hpp` (sidescan_core, header-only), used by both the session
   and the new reader. Pure/deterministic — unit-testable.

1b. **Cross-bag frame consistency (design decision)** — each bag's `world_frame`
   is a LOCAL map frame whose origin is not guaranteed identical across
   recordings. Combining passes from different bags therefore reprojects
   through the geo anchor: the reader captures earth<-world (the same
   transform `SidescanBagSession` captures for `mapToGeo()`); the cloud window
   picks the FIRST pass's bag as the reference frame and reprojects every
   other bag's soundings via `T_ref<-earth * T_earth<-src` (one composed
   transform per bag, applied per point — pure helper in
   `mbes_window_reader.hpp`, unit-testable with synthetic transforms). A bag
   with no earth<-world available: if its world frame NAME matches the
   reference's, use identity (same-recording case); otherwise skip the pass
   and count it visibly in the window status — never silently mis-place
   soundings.

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

5. **Tests** — follow the package norm: pure-math units + real-data smoke, no
   synthetic-bag fixture (no existing test writes a rosbag; bag glue is thin by
   construction here because projection/lift/reprojection are all pure).
   - `test/test_mbes_window_reader.cpp` (NEW, pure parts only): the cross-bag
     reprojection composition (synthetic earth<-ref / earth<-src transforms →
     known point mapping, including the identity same-frame case) and the
     `tf_lift.hpp` helpers (`rotate_by_quat` against known rotations;
     `lookup_at_or_latest` fallback behaviour with a hand-filled BufferCore).
   - `test/test_point_cloud_view.cpp` (EXTEND): `MultiPassColour` — two passes,
     distinct golden-angle hues, rendered pixels non-background (mirrors
     `RendersAndClears`); `ColorModePass` setter safety; pass-hue assignment
     function pinned for the first few indices.
   - **Real-data smoke** (verification step, not a committed test): overview →
     multi-select passes over a Massabesic spot → cloud window renders, status
     reports pass/sounding counts, offscreen.

6. **CMakeLists.txt** — add `src/mbes_window_reader.cpp` to `sidescan_core`;
   add `src/mbes_cloud_window.cpp` to `sidescan_target_viewer` sources; add
   `test_mbes_window_reader` gtest linked against `sidescan_core`.

7. **README update** — add a "Survey explorer stage 3" section under the existing
   `sidescan_target_viewer` notes describing the multi-pass MBES cloud feature.

## Files to Change

| File | Change |
|------|--------|
| `src/mbes_window_reader.hpp` | NEW (sidescan_core) — `read_mbes_window` + reprojection helper |
| `src/mbes_window_reader.cpp` | NEW (sidescan_core) — filtered/seek windowed read + TF2 lift |
| `src/tf_lift.hpp` | NEW (sidescan_core) — `lookup_at_or_latest` + `rotate_by_quat` extracted |
| `src/sidescan_bag_session.cpp` | Use `tf_lift.hpp` instead of its anon-namespace copies |
| `src/mbes_cloud_window.hpp` | NEW — `MbesCloudWindow` declaration |
| `src/mbes_cloud_window.cpp` | NEW — multi-pass 3D viewer window |
| `src/point_cloud_view.hpp` | Add `ColorMode::Pass`, `setMultiPassPoints()` |
| `src/point_cloud_view.cpp` | Implement `ColorMode::Pass` in `rebuild_colors()`, `setMultiPassPoints()` |
| `src/survey_overview_window.hpp` | Add `onViewMbesCloud()` slot |
| `src/survey_overview_window.cpp` | Extended-select, "View MBES cloud" button, `onViewMbesCloud` |
| `test/test_mbes_window_reader.cpp` | NEW — pure-math units (reprojection, tf_lift helpers) |
| `test/test_point_cloud_view.cpp` | Extend with `MultiPassColour` + `ColorModePass` tests |
| `CMakeLists.txt` | New sources into sidescan_core + viewer; `test_mbes_window_reader` target |
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

Single PR. 14 files new/modified (upper single-PR bound, but one cohesive
vertical slice: reader → color mode → window → overview wiring). The most
complex new piece is `mbes_window_reader.cpp` (filtered/seek windowed read +
TF2 lift + cross-bag reprojection). All changes are in
`marine_perception_tools`.
