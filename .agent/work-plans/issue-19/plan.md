# Plan: Survey Overview Pane for sidescan_target_viewer

## Issue

https://github.com/rolker/marine_perception_tools/issues/19

## Context

The jump-to-pass bridge (#17) and the survey index+query CLI (unh_marine_autonomy#259)
are merged. The viewer still needs a visual entry point: a survey-wide map showing the
tiled bathymetry basemap, click-to-query against `survey_index.db`, and pass selection
opening the existing `SidescanViewerWindow`. Launching with `--index` (no bag) should
show the map and let the operator click any location to see which passes cover it.

## Approach

1. **`SurveyIndexBridge` (headless)** — `survey_index_bridge.hpp/cpp`: open
   `survey_index.db` via SQLite, expose `queryPasses(lat, lon)` → `std::vector<PassRow>`.
   Internally calls `marine_survey_index::tilesForBoundingBox` at L14 to get candidate
   tiles, then `marine_survey_index::queryPasses`. No Qt dependency; unit-testable with
   an in-memory DB.

2. **`SurveyOverviewCanvas` (Qt widget)** — `survey_overview_canvas.hpp/cpp`: sibling
   to `SidescanCanvas`. Renders georeferenced tile images in lat/lon space with a
   `cos(lat)` column scale factor for aspect-ratio correction (north-up; row 0 = north).
   Accepts zoom/pan (wheel + drag). Emits `clicked(double lat, double lon)` on left
   click. Accepts `std::vector<QImage + bounds>` tiles from the owner.

3. **`SurveyOverviewWindow`** — `survey_overview_window.hpp/cpp`: top-level `QMainWindow`
   holding the canvas (left) and a pass-list panel (right splitter). On construction,
   scans `--stores <dir>` for `*.tif` files with `marine_tiled_raster_store::loadTiles<double>`
   (bathymetry layer, single-band depth → colormapped QImage). Connects canvas click to
   `SurveyIndexBridge::queryPasses`; populates the pass list (`QTreeWidget`) grouped by
   bag path with columns: time window, duration, ping count, sensor. Double-clicking
   a pass row calls `openViewer(bag, t_start_ns, t_end_ns)` which constructs or reuses
   a `SidescanViewerWindow` and calls `openBag()`.

4. **CLI additions to `sidescan_viewer_main.cpp`** — add `--stores <dir>` and
   `--index <path>` options. When `--index` is given, open `SurveyOverviewWindow` as
   the primary window (bag argument is optional). Without `--index`, behaviour is
   unchanged.

5. **CMakeLists.txt** — add `marine_survey_index` and `marine_tiled_raster_store` to
   `find_package` and link against the `sidescan_target_viewer` executable. Add SQLite3
   via `find_package(SQLite3 REQUIRED)`.

6. **Unit tests** — `test_survey_index_bridge.cpp`: open an in-memory SQLite DB,
   populate schema manually, call `queryPasses(lat, lon)`, assert expected `PassRow`
   results. No bag I/O. Pattern follows `marine_survey_index`'s `test_query_join.cpp`.

## Decisions (plan review, 2026-07-14)

- **package.xml must be updated alongside CMakeLists** (review must-fix):
  `<depend>marine_survey_index</depend>`, `<depend>marine_tiled_raster_store</depend>`,
  `<depend>libsqlite3-dev</depend>` — without them rosdep/ament resolution breaks.
- **The lat/lon→pixel projection is a pure, unit-tested function** (review
  suggestion): `survey_overview_projection.hpp` (header-only, Qt-free) owns the
  cos(lat)-scaled geo↔pixel mapping; `SurveyOverviewCanvas` consumes it. New
  `test_survey_projection.cpp` covers round-trip, aspect correction at the
  survey latitude, and north-up orientation.
- **SQLite3 is linked, not just found**: `find_package(SQLite3 REQUIRED)` +
  `target_link_libraries(... SQLite::SQLite3)` on the executable (the bridge
  compiles into it).

## Files to Change

| File | Change |
|------|--------|
| `src/survey_index_bridge.hpp` | New: headless SQLite query wrapper |
| `src/survey_index_bridge.cpp` | New: implementation of `queryPasses(lat, lon)` |
| `src/survey_overview_canvas.hpp` | New: lat/lon tile-map widget |
| `src/survey_overview_canvas.cpp` | New: paint + zoom/pan + click signal |
| `src/survey_overview_window.hpp` | New: overview QMainWindow with pass list |
| `src/survey_overview_window.cpp` | New: tile load, index query, viewer hand-off |
| `src/sidescan_viewer_main.cpp` | Add `--stores`/`--index` CLI opts; conditionally open overview |
| `CMakeLists.txt` | Add deps: `marine_survey_index`, `marine_tiled_raster_store`, SQLite3 |
| `test/test_survey_index_bridge.cpp` | New: in-memory DB unit test for query bridge |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Eager tile load only (Massabesic is small); no speculative multi-layer UI |
| Improve incrementally | One new executable window; existing per-bag flow untouched |
| Test what breaks | Bridge tested with in-memory DB; avoids bag I/O in CI |
| A change includes its consequences | CMakeLists, CLI, and tests updated in same PR |
| Human control and transparency | `--index` opt-in; no hidden automation |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0008 — ROS 2 conventions | Yes (new source files in ROS package) | Follow naming, license headers, ament macros |
| 0003 — Project-agnostic workspace | No — changes are in project repo only | N/A |
| 0013 — progress.md vocabulary | Yes (this entry) | `## Plan Authored` per vocabulary |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `sidescan_viewer_main.cpp` CLI args | `README.md` usage section | Yes — update README in same PR |
| Add new executables/libraries | `install(TARGETS ...)` in CMakeLists | Yes |

## Open Questions

- **Tile load path**: `--stores <dir>` is the root; the bathy layer lives one level
  below (e.g. `<stores>/bathymetry/<level>_<row>_<col>.tif`). Confirm the expected
  subdirectory name before implementation (`marine_bathymetry_store` naming convention).
- **Colormap for depth**: Which `marine_colormap` palette to use by default for the
  bathy basemap (e.g. `jet`, `viridis`, or the existing palette combo from the viewer)?

## Estimated Scope

Single PR.
