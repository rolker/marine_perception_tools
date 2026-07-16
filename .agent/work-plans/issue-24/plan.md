# Plan: Integrated explorer shell — single main window, tile-granular map selection, nav track, rename

## Issue

https://github.com/rolker/marine_perception_tools/issues/24

## Context

`sidescan_target_viewer` currently opens `SurveyOverviewWindow` (tile-map + pass list) and
`SidescanViewerWindow` / `MbesCloudWindow` as separate top-level windows. The survey overview
uses point-click (`clicked(lat,lon)`) to query passes. There is no nav-track overlay.

This PR restructures the explorer into a single `QMainWindow` with docked panes, replaces
point-click with tile-granular rubber-band selection, overlays the v2 nav track, and renames
the primary executable to `survey_explorer` (name is a checkpoint decision below).

Dependency on `unh_marine_autonomy#265` is **resolved**: `queryNavTrack` and
`queryNavTrackInBox` live in `marine_survey_index/query.hpp`; the production index already
holds v2 data (46,541 points, 31 bags). V1 DBs throw version mismatch at `openIndexDb` (the
viewer already surfaces that); bags with no posed pings produce no rows — draw nothing.

## Approach

1. **Rename executable + add shim** — rename `sidescan_target_viewer` → `survey_explorer`
   (name checkpoint below); add a thin `sidescan_target_viewer_shim.cpp` whose `main()`
   calls `execv(sibling-path/survey_explorer, argv)` so any scripted caller using the old
   name keeps working. Update `CMakeLists.txt` install rule, `package.xml` description,
   and `README.md` (add new top-level section, update CLI examples).

2. **Single main window** — rename `SurveyOverviewWindow` → `SurveyExplorerWindow`. On
   double-click in the pass list, host the `SidescanViewerWindow` inside a `QDockWidget`
   docked at the bottom of `SurveyExplorerWindow` rather than spawning a separate top-level
   window. Same pattern for `MbesCloudWindow`: docked at the right. Each dock carries a
   title and a close button; closing a dock frees its window (`WA_DeleteOnClose` on the dock
   widget itself). Multiple sidescan docks can coexist (one per activation, tabbed).

3. **Tile-granular selection in `SurveyOverviewCanvas`** — replace the point-click signal
   `clicked(lat,lon)` with tile-selection events:
   - Regular left-drag pans (unchanged).
   - Ctrl-click toggles the tile under the cursor.
   - Ctrl-drag rubber-bands a pixel rect; on release, all tiles whose pixel rects intersect
     the rubber band are added to the selection.
   - Selected tiles are drawn with a semi-transparent cyan tint (same colour as the query
     crosshair, 30 % alpha overlay on the tile rect).
   - New signal: `tileSelectionChanged(std::vector<std::size_t> selected_indices)` — indices
     into the current `tiles_` vector.
   - `SurveyIndexBridge` gains `queryTileBox(south, west, north, east)` that expands the
     union of selected-tile bounds and calls `tilesForBoundingBox` + `queryPasses` (same
     as the existing `queryPoint` internals, without the radius offset). The pass-list build
     in `SurveyExplorerWindow` connects to `tileSelectionChanged` instead of `onMapClicked`.

4. **Nav track overlay** — add `queryAllNavTrack()` to `SurveyIndexBridge` (raw SQL
   `SELECT bag_id, t_ns, latitude, longitude FROM nav_track ORDER BY bag_id, t_ns`; returns
   empty vector when the table has no rows). `SurveyExplorerWindow` calls this after
   `loadStoreTiles`, passes the result to `canvas_->setNavTrack(points)`. The canvas draws
   per-bag polylines (segment at bag_id changes, as specified in the query contract). At
   intervals of ~30 display pixels an arrowhead is drawn in the direction of travel between
   successive time-ordered points, pointing from older to newer. Track colour: white at 60 %
   alpha so it reads over both dark basemap and bright tiles. No speed filtering — render
   faithfully including the three 2026-06-26 oscillating-FCU bags (their tracks will look
   scribbled; that's correct).

5. **Update docs** — add a `## survey_explorer` top-level section in `README.md` under the
   new name, covering the single-window layout, tile selection, and nav-track overlay. Keep
   the existing `sidescan_target_viewer` mention with a one-liner noting it is a backward-
   compatible shim. Update `package.xml` description.

## Files to Change

| File | Change |
|------|--------|
| `src/sidescan_viewer_main.cpp` | Rename → `src/survey_explorer_main.cpp`; update `QApplication::setApplicationName` to `"survey_explorer"` |
| NEW `src/sidescan_target_viewer_shim.cpp` | Thin `main()` that `execv`s the sibling `survey_explorer` binary |
| `src/survey_overview_window.hpp` + `.cpp` | Rename class `SurveyOverviewWindow` → `SurveyExplorerWindow`; host sidescan + cloud views as `QDockWidget` panes; connect to `tileSelectionChanged`; call `queryAllNavTrack` and `setNavTrack` |
| `src/survey_overview_canvas.hpp` + `.cpp` | Add ctrl-click + rubber-band tile selection; add nav-track draw; replace `clicked` signal with `tileSelectionChanged`; add `setNavTrack(vector<NavPoint>)` |
| `src/survey_index_bridge.hpp` + `.cpp` | Add `queryTileBox(south, west, north, east)` and `queryAllNavTrack()` |
| `CMakeLists.txt` | Rename `sidescan_target_viewer` target → `survey_explorer`; add shim target; update `install(TARGETS ...)` |
| `package.xml` | Update description to name `survey_explorer` as the primary tool |
| `README.md` | Add `## survey_explorer` section with layout, tile-selection, and nav-track docs; note the shim |
| `test/test_survey_index_bridge.cpp` | Add fixture rows to `nav_track`; add tests for `queryAllNavTrack` (populated and empty) and `queryTileBox` (non-empty tile selection) |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Single main window is more discoverable than multiple independent windows. Tile tinting makes the selection state visible. QDockWidget layouts are user-resizable and their positions persist (QSettings). |
| Only what's needed | Tile-granular selection replaces point-click directly; no intermediate abstraction. Nav track uses the existing query API — no new index schema. |
| Improve incrementally | Four items land together because they are a coherent restructure (the docking depends on the rename; tile selection changes the same signal the dock connection reads). |
| A change includes its consequences | Rename consequences enumerated in the Files table; shim preserves backward compat; README and package.xml updated in the same PR. |
| Test what breaks | `test_survey_index_bridge` gains `queryTileBox` and `queryAllNavTrack` coverage. The tile-selection interaction path is tested through the bridge (headless). The dock re-parenting is smoke-tested via the existing `test_point_cloud_view` offscreen pattern — add a `test_survey_explorer_window` offscreen smoke test for the dock-open path. |
| Capture decisions, not just implementations | Rename rationale and QDockWidget strategy documented here; shim strategy recorded in CMakeLists comment. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0001 (Adopt ADRs) | Watch | Rename decision is captured here; no new ADR required unless the naming convention deviates from ROS 2 package conventions (it doesn't — `survey_explorer` is plain snake_case). |
| ADR-0002 (Worktree isolation) | OK | Worktree `issue-marine_perception_tools-24` is active on `feature/issue-24`. |
| ADR-0008 (ROS 2 conventions) | Yes | CMakeLists.txt install rule uses `lib/${PROJECT_NAME}` (already correct). `package.xml` description updated. No new `exec_depend` entries — local executables are not `exec_depend`. |
| ADR-0013 (progress.md vocabulary) | OK | This entry uses `## Plan Authored`. |

## Consequences

| If we change… | Also update… | Included in plan? |
|---|---|---|
| Rename `sidescan_target_viewer` executable | `CMakeLists.txt` install, `package.xml` description, `README.md` CLI section | Yes |
| `SurveyOverviewWindow` → `SurveyExplorerWindow` | `#include` in `survey_explorer_main.cpp`; forward declarations in any file that names it | Yes — same PR |
| `clicked(lat,lon)` signal removed from canvas | `SurveyExplorerWindow` slot `onMapClicked` replaced by `onTileSelectionChanged` | Yes |
| `SidescanViewerWindow` hosted in a dock | `onPassActivated` in `SurveyExplorerWindow` changed; no external callers (main.cpp only used it as a standalone window, now as a dock pane) | Yes |

## Open Questions

- **Rename target name**: plan proposes `survey_explorer`. Roland has not fixed the name — needs confirmation before implementation. (Checkpoint: approve or supply an alternative before step 1.)
- **Dock strategy for `SidescanViewerWindow`**: proposal wraps `SidescanViewerWindow : QMainWindow` in a `QDockWidget` (Option A — lower refactor risk). If Roland prefers a cleaner embedding, `SidescanViewerWindow` should be refactored to `QWidget` (Option B — follow-up). Flag if Option A's nested menu bar is unacceptable.

## Estimated Scope

Single PR. ~8–10 files changed, 2 new source files. Moderate complexity — the tile-selection
canvas change and QDockWidget re-parenting are the most involved pieces.
