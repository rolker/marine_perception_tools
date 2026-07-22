# Plan: Integrated explorer — grow SidescanViewerWindow into the survey explorer

## Issue

https://github.com/rolker/marine_perception_tools/issues/24

## Context (corrected at plan checkpoint, 2026-07-16)

The first plan draft proposed a new shell window hosting the existing windows as
`QDockWidget`s. **Roland corrected the architecture**: `SidescanViewerWindow`
already has the right panes (geo canvas, sidescan waterfall, MBES waterfall,
3D point cloud, contact list) — the work is to *improve those panes* and absorb
the scaffolding windows (`SurveyOverviewWindow`, `MbesCloudWindow`), not to nest
windows inside a new shell.

**Roland's flow (2026-07-16, verbatim intent):**
- The geo pane is the **index map**: existing store imagery (bathy/sidescan
  GeoTIFF tiles) as basemap, nav track with direction arrows, selectable index
  tiles, the current pass's georeferenced sidescan on top, contact marks.
- Selecting one or more tiles loads **all bags' data** for the point-cloud /
  (future stage-4) CUBE-surface display — automatically, across bags, per-pass
  colours. No manual pass-picking step for the cloud.
- From those passes, a **timeline** decides what the single-pass waterfall
  displays: the selection's pass intervals drawn on a time axis; activating an
  interval cues the waterfall via the existing bag-open + cue machinery.
- The waterfall stays single-pass by design (blending kills shadows).

Dependency `unh_marine_autonomy#265` is **resolved and deployed**: schema v2
`nav_track` + `queryNavTrack`/`queryNavTrackInBox` accessors; the production
index is regenerated (46,541 points / 31 bags). `marine_survey_index_core` is a
**static** lib — this package must rebuild against the v2 underlay (host builds).

## Approach

1. **Unified geo canvas (index map).** Merge the two canvases into one
   **geographic-frame** canvas and retire `SurveyOverviewCanvas`. Frame
   correction (review-plan finding): `SidescanCanvas` is a per-bag **map-ENU
   (metres)** canvas; the geographic `GeoView`/`geoToPixel` projection lives in
   `survey_overview_projection.hpp` (pure, unit-tested — **retained and
   reused**, along with `test_survey_projection.cpp`). The merged canvas adopts
   the geographic frame; the per-bag sidescan coverage image and contact
   overlays are reprojected map-ENU→geo via the existing
   `SidescanBagSession::mapToGeo` anchor (already used for contact geometry at
   sidescan_viewer_window.cpp:981). Layer order, bottom-up: store tiles → nav track
   (per-bag polylines, arrowheads every ~30 px, white 60 % alpha, segment at
   bag_id changes) → index-tile grid + selection tint → current-pass sidescan
   coverage image → contacts/crosshair. Interaction:
   - Left-drag pans, wheel zooms (unchanged).
   - Ctrl-click toggles the index tile under the cursor; ctrl-drag rubber-bands
     tiles (intersection with pixel rect).
   - New signal `tileSelectionChanged(std::vector<gggs::GridIndex>)`.
   - Selection-tint + hit-test math in a pure header
     (`tile_selection.hpp`-style) so it unit-tests without a GL/Qt context.
   - Keep the stage-2 fallback-fit behaviour (index extent when no store tiles).

2. **Index integration in the viewer.** Move `--index`/`--stores` handling from
   `SurveyOverviewWindow` into `SidescanViewerWindow`: it owns a
   `SurveyIndexBridge`, loads store tiles + nav track at startup — the
   all-bags track comes from the query library, not raw SQL:
   `queryNavTrackInBox(extent())` using the bridge's existing `extent()`
   (review-plan finding; keeps the bridge's reuse-the-library contract), and on
   `tileSelectionChanged` runs `queryPasses` over the selected tiles (new bridge
   helper `queryTiles(vector<GridIndex>)` — the tiles are already index keys, no
   bounding-box detour). Bag-only invocation (no `--index`) keeps working: map
   shows just the per-bag coverage image as today.

3. **Multi-pass cloud in place.** Port `MbesCloudWindow`'s loader
   (QtConcurrent + `read_mbes_window` + earth-anchor reprojection + per-pass
   golden-angle colours + legend + skip notes) into the viewer's existing cloud
   pane (`PointCloudView` is already shared). On selection change, the cloud
   auto-loads **all** `mbes-bathy` passes of the selected tiles across bags
   (cancel/supersede a stale in-flight load: track a generation counter; apply
   results only if current). Legend lives beside the cloud pane. Retire
   `MbesCloudWindow`.

4. **Pass timeline pane (new widget).** `PassTimelineWidget`: the selection's
   pass intervals on a horizontal UTC time axis, one row per sensor type
   (mbes / sidescan-port / sidescan-stbd), bars coloured with the same
   golden-angle pass colours as the cloud legend (shared `pass_color`).
   Gaps between survey days compress (the axis maps only the covered spans,
   with visual break markers) so a three-week campaign stays scrubbable.
   Clicking a bar cues the waterfall: existing `openBag(bag, t_start, t_end)`
   machinery (bag switch if needed). Tooltip: bag basename, UTC interval,
   duration, ping count. Pure time-axis mapping (interval → x-range,
   compressed-gap model) in a testable header. The timeline replaces the
   stage-2 pass list (the tooltip + a status readout carry the detail the list
   showed).

5. **Rename + retire scaffolding.** Executable → `survey_explorer`
   (**Roland-confirmed at checkpoint 2026-07-16**); `sidescan_target_viewer`
   remains as a shim (`execv` of the sibling binary). Retire
   `SurveyOverviewWindow` (+ its canvas) — `--index` and bag modes converge on
   the one window. Update `CMakeLists.txt` (targets, install), `package.xml`
   description, `README.md` (new top-level section under the new name; shim
   note), `.agents/README.md` (inventory + viewer row). Closes the #22 timing
   question (comment there after merge).

## Files to Change

| File | Change |
|------|--------|
| `src/sidescan_canvas.hpp/.cpp` | Adopt geographic frame (reuse `survey_overview_projection.hpp`); absorb overview layers: store tiles, nav track, tile grid + selection, fallback fit; reproject per-bag coverage via `mapToGeo`; `tileSelectionChanged` signal |
| NEW `src/tile_selection.hpp` | Pure hit-test / rubber-band-intersection / selection-set math |
| NEW `src/map_geo_anchor.hpp` | `MapGeoAffine` + the pure `probe_map_anchor` (moved out of the canvas header so the probe unit-tests without Qt) |
| NEW `src/pass_coalesce.hpp` | Pure per-tile→per-physical-pass segment coalescing, extracted from the stage-2 overview window; shared by the timeline and the cloud loader |
| NEW `src/basemap_contrast.hpp` + basemap controls (desk-verify follow-up, 2026-07-22) | Roland's desk check found the basemap flat (residual store outlier cells owned the min/max colour scale) and asked for basemap colormap + store-layer controls. Added: percentile contrast (`robust_range`), a Map pane header with layer picker (layers discovered under the stores root: bathymetry/backscatter/sidescan) + colormap combo, async basemap loading with a generation counter, an image-memory budget (large sidescan layers decimate), and 0-as-NoData for the uint16 sidescan composites |
| NEW `src/mbes_pass_loader.hpp/.cpp` | Multi-pass cloud loader extracted from `MbesCloudWindow` (single source of truth until its phase-e retirement) |
| NEW `src/pass_timeline_widget.hpp/.cpp` | Timeline pane: rows per sensor, gap-compressed UTC axis, pass bars, activate signal |
| NEW `src/pass_timeline_model.hpp` | Pure interval → x mapping with gap compression (unit-testable) |
| `src/sidescan_viewer_window.hpp/.cpp` | Own SurveyIndexBridge; wire tile selection → cloud auto-load + timeline; embed legend; layout: timeline row under the scrub controls |
| `src/point_cloud_view.*` | (already has multi-pass API from stage 3 — no change expected) |
| `src/survey_index_bridge.hpp/.cpp` | Add `queryTiles(vector<GridIndex>)`, `queryAllNavTrack()` |
| DELETE `src/survey_overview_window.*`, `src/survey_overview_canvas.*`, `src/mbes_cloud_window.*` | Absorbed (`survey_overview_projection.hpp` + its test are RETAINED) |
| `src/sidescan_viewer_main.cpp` → `src/survey_explorer_main.cpp` | Rename; `--index`/`--stores` route into the one window |
| NEW `src/sidescan_target_viewer_shim.cpp` | Back-compat shim |
| `CMakeLists.txt` | Target renames, shim, deletions |
| `package.xml`, `README.md`, `.agents/README.md` | Docs under the new name |
| `test/test_survey_index_bridge.cpp` | `queryTiles`, `queryAllNavTrack` (populated + empty) |
| NEW `test/test_tile_selection.cpp` | Hit-test + rubber-band math pins |
| NEW `test/test_map_geo_anchor.cpp` | Probe recovers a synthetic affine mapping; partial/non-finite probes yield no anchor |
| NEW `test/test_mbes_pass_loader.cpp` | Loader error paths: empty input, missing bag costs only its own pass |
| NEW `test/test_pass_timeline_model.cpp` | Gap compression, interval mapping, activation lookup pins |
| existing window/GL tests | Update for renamed class/retired windows; NEW `test_survey_explorer_window.cpp` offscreen smoke asserting the `--index` path constructs, loads tiles+track, and survives a synthetic tile selection |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | One window, selection state visible on the map, timeline shows exactly what feeds the waterfall; cloud legend + timeline share colours so "which pass is which" is answerable at a glance. |
| Only what's needed | No new shell; scaffolding windows retire; pure-math headers only where testability demands. |
| Improve incrementally | Commit-phased inside one PR: (a) bridge accessors + pure headers + tests, (b) canvas merge, (c) cloud in place, (d) timeline, (e) rename/docs/retire. Each commit builds green. |
| A change includes its consequences | Rename consequence list enumerated; retired files removed from CMake; docs updated in-PR; #22 closed by this. |
| Test what breaks | Selection math, timeline model, and bridge accessors get dedicated tests; GL/window paths get offscreen smoke tests (existing pattern). |

## Consequences

| If we change… | Also update… | Included? |
|---|---|---|
| Retire SurveyOverviewWindow/Canvas + MbesCloudWindow | CMakeLists sources, any tests naming them, `.agents/README.md` layout table | Yes |
| `clicked(lat,lon)` → tile selection | Stage-2 click-to-query flow is subsumed (tile toggle = the click) | Yes |
| Executable rename | CMake install, package.xml, README, .agents/README, shim | Yes |
| Cloud auto-load on selection | Generation-counter cancellation so stale loads never clobber a newer selection | Yes |

## Open Questions

None — both checkpoint items resolved by Roland 2026-07-16:
- **Executable name**: `survey_explorer` (confirmed).
- **Timeline vs pass list**: timeline only (confirmed); tooltips + status
  readout carry the detail.

## Estimated Scope

Single PR, commit-phased (5 phases above). If the PR balloons, the
`PassTimelineWidget` (phase d) is the designated spill — self-contained and
additive, it can split into an immediate follow-up without breaking the arc. ~16 files (3 new sources, 3 new
tests, 3 retired). The canvas merge and the selection→cloud auto-load wiring are
the most involved pieces.
