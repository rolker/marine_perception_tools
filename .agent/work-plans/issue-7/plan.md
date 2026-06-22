# Plan: Offline georeferenced sidescan viewer + manual target tracker

## Issue

https://github.com/rolker/marine_perception_tools/issues/7

## Context

OpenSidescan does not fit our workflow (no ROS 2 / `RawSonarImage` / MCAP ingest,
weak georeferencing, performance). Build our own offline viewer as a **second app**
in this package, mirroring the existing `tuner_core` (Qt-free, unit-tested) +
`sea_surface_tuner` (Qt app) split. Driver: Lake Massabesic submerged-object search —
analyze recorded bags, mark contacts, carry them across passes.

Strong in-repo reuse: `BagSession` (scan → TF cache + bounds → windowed reads) is a
near-exact template for the sidescan reader; `buffer_policy.hpp` (time-windowed span
policy) maps to a distance-windowed analogue; `cv_qt.hpp` for `cv::Mat`→`QImage`.
`tf2`, `image_geometry`, `grid_map_core`, `rosbag2_cpp` are already deps.

## Approach (v1 = critical path; echogram + store-swap are fast-follow)

1. **Scaffold** — add `sidescan_core` lib + `sidescan_target_viewer` executable to
   CMakeLists/package.xml. New deps: `marine_interfaces` (Contact), `marine_acoustic_msgs`
   (RawSonarImage), `geographic_msgs` + `geodesy` (export-boundary geo).
2. **Ingest** — `SidescanBagSession` (mirror `BagSession`): scan builds TF cache + nav
   timeline (`/bizzy/odom` + TF) + per-channel `RawSonarImage` ping index +
   cumulative along-track distance; windowed read **by distance** (not time). Verify
   `earth → bizzy/map` presence on scan (geo-export readiness).
3. **Projection (pure, unit-tested)** — per-ping sensor pose in `bizzy/map` from TF;
   altitude from the driver's `nadir_depth` (`sensor_msgs/Range`), amplitude
   first-return estimator as fallback; slant→ground-range; across-track sample →
   `bizzy/map` XY.
4. **Swath render + coverage** — paint port+starboard ground-range samples into an
   in-app `bizzy/map`-frame raster (`grid_map_core`); **quality-wins** per cell
   (grazing-angle/range metric); coverage mask → **skip fully-covered pings**; distance
   buffer policy (distance analogue of `buffer_policy.hpp`) with **stationary ping
   cap**. Cap + coverage-skip thresholds are exposed as tunable params, not hardcoded.
5. **Map canvas (Qt)** — north-up `QGraphicsView` in map meters; **customizable
   measuring grid**; **zoom**; render swath raster + boat track; **distance scrubber**.
6. **Contact marking** — box-drag → `Contact` (`ORIGIN_HUMAN`, `STATUS_PROPOSED`,
   `existence_probability=1.0`, single `Classification` @ 1.0, `Shape.BOX` dims from box
   extent, shadow→height, `geo_pose` **resolved** via `earth → bizzy/map` TF + geodesy
   — never left NaN, `source="sidescan.port"`); thumbnail crop; target-list pane (table
   + thumbnail), click-to-locate.
7. **Store + cross-pass overlay** — thin `ContactArray` file store (forward-compatible
   with contact_manager #167); load on open → in-memory spatial index; reverse-project
   in-view contacts (from any pass) onto the canvas.
8. **Tests** — distance-buffer policy, projection math, quality-wins/coverage, store
   round-trip (all in the Qt-free core).

## Files to Change

| File | Change |
|------|--------|
| `package.xml`, `CMakeLists.txt` | Add core lib + app target + new deps |
| `src/sidescan_bag_session.{hpp,cpp}` | Distance-indexed `RawSonarImage`+nav reader |
| `src/sidescan_projection.{hpp,cpp}` | Slant→ground, across-track→map-XY (pure) |
| `src/distance_buffer_policy.hpp` | Distance analogue of `buffer_policy.hpp` |
| `src/coverage_raster.{hpp,cpp}` | lean self-contained quality-wins raster + coverage skip (no grid_map) |
| `src/contact_store.{hpp,cpp}` | `ContactArray` load/save + spatial index |
| `src/sidescan_main_window.{hpp,cpp}`, `src/sidescan_viewer_main.cpp` | Qt app |
| `test/test_*.cpp` | Core unit tests |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Quality Standard (robustness, tests) | Breakable logic kept Qt-free + unit-tested; explicit handling of TF gaps, empty windows, stationary boat, fully-covered pings |
| Reuse over reinvent | Mirrors `BagSession`/`buffer_policy`/`cv_qt`; store = merged `Contact` msg, no bespoke schema |
| Documentation accuracy | Topic names/msg fields verified against real bags + merged `Contact.msg` |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| Sidescan store (ADR-0005/0006, uma) | Partial | We don't implement the durable store; we produce forward-compatible `Contact`s |
| GeoCoder incidence (ADR-0007) | No | v1 uses flat-bottom nadir altitude; no GeoCoder coupling |
| Contact model (uma#167/#157) | Yes | Consume merged `Contact`/`ContactArray`; swap to its CRUD API as fast-follow |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| Add app to this package | `package.xml`/CMake deps, README | Yes |
| Produce `Contact` files | contact_manager #167 ingest seam | Follow-up (fast-follow) |

## Decisions (resolved with Roland, 2026-06-21)

- **GGGS descoped for v1.** Paint the swath into an in-app map-frame raster
  (`grid_map_core`); use geodesy only at the export boundary (map-XY → lat/lon). No
  GGGS/Web-Mercator tile stack for display. GGGS tile-store sharing is a fast-follow.
- **Store format = CDR.** Serialize `ContactArray` as CDR (exact message fidelity,
  closest to the contact_manager #167 seam).
- **Frames (verified against bag `bizzyboat_sonar/2026-06-18T19-39-06+00-00`):** render
  in **`bizzy/map`** (this bag's `tf_static` carries `bizzy/map → map`); odom =
  **`bizzy/odom` → `bizzy/base_link`**; sensors = **`bizzy/garmin_sidescan_{port,
  starboard,down}`**. NOT the tuner's `bizzy/map_tide`. Geo export needs
  `earth → bizzy/map` — PR1 ingest verifies its presence; datum fallback if absent.
- **`marine_acoustic_msgs` confirmed** installed at `/opt/ros/jazzy` (binary dep) —
  PR1 compiles. Add as `<depend>`.
- **Altitude = driver `nadir_depth` (`sensor_msgs/Range`)** — the Garmin driver's
  bottom-tracked height-above-bottom, the authoritative source (verified 5.9–15.2 m on
  the real bag); the down-channel amplitude first-return estimator is a fallback only.
  Port + starboard are the painted swath. (Resolves the down-look ambiguity.)

## Open Questions

- **Echogram (fast-follow):** `rqt_marine_sonar` has a C++ Qt echogram widget — reuse by
  extraction vs reimplement. Decide when we start that phase.
- **`earth → bizzy/map` availability:** present in these bags? PR1 ingest verifies; if
  absent, geo_pose export needs a datum fallback (lake datum 52.3 m WGS84 known).

## Implementation Notes

- **PR1 landed** (`sidescan_core`): `sidescan_geometry.hpp` (pure, 9 gtests), `SidescanBagSession`
  ingest, `sidescan_probe` CLI. Verified on `bizzyboat_sonar/2026-06-18T19-39-06+00-00`:
  48 551 pings (port/stbd/down balanced), 676 m track, `earth→bizzy/map` available,
  ~24% of pings have no TF pose at their stamp (early-bag localization warm-up; pings
  retained but unpaintable — expected).
- **Altitude source switched to the driver's `nadir_depth` Range** (Roland's call): the
  amplitude estimator fired at an implausible ~0.1 m (near-field ringing) on the real
  bag, whereas `nadir_depth` gives the driver's bottom-tracked depth directly
  (3.62 m on the probe ping; 5.9–15.2 m across the bag). The amplitude estimator is
  retained as a fallback (with its 9 gtests) for bags/sonars lacking the topic.
  `SidescanBagSession::usedNadirDepth()` reports which path was taken.

- **PR2 (in progress): Qt-free core landed.** `distance_buffer_policy.hpp` (rolling
  distance window + stationary keep-cap; 6 gtests) and `coverage_raster.{hpp,cpp}`
  (quality-wins paint, coverage tracking, `ping_covered_fraction` for skip-fully-covered,
  `paint_ping`; 6 gtests). **Deviation:** used a lean self-contained raster instead of
  `grid_map_core` — deterministic to unit-test, no Eigen-heavy dep in the test, and the
  paint/quality-wins/coverage logic is the part we most want under test. Reversible if
  we later want grid_map interop. Quality metric is a range proxy (`range_quality`),
  a placeholder for a grazing-angle metric.
- **PR2 (Qt canvas) landed — first runnable `sidescan_target_viewer`.**
  `sidescan_canvas.{hpp,cpp}` (north-up custom-QWidget map render in `bizzy/map` metres:
  coverage QImage at true position + boat-track polyline + measuring grid; wheel zoom
  about cursor, drag pan), `sidescan_viewer_window.{hpp,cpp}` (File→Open bag, distance
  scrubber, grid-spacing + window-length spins, status line), `sidescan_viewer_main.cpp`.
  Per scrub it paints the rolling window's port+stbd pings (stationary-capped) into a
  bbox-sized `CoverageRaster`, renders grayscale, shows it at map position. Verified:
  builds, 180 tests pass, lint clean, **headless offscreen run on the real bag loaded +
  rendered + ran the event loop ~18 s, no crash**. Awaiting Roland's visual check.
  Placeholders: synchronous bag load (async/QtConcurrent is a follow-up like the tuner);
  grayscale render (marine_colormap shader later). **Remaining:** visual tuning →
  contact marking + Contact store + cross-pass overlay (PR3).

- **Windowed re-read landed (memory bound).** Replaced the all-samples-in-RAM index
  with a lightweight index (stamp/distance/pose/geometry, no samples) + `readWindow()`
  that re-reads only the current distance window's samples from the bag via
  `Reader::seek` (matched back to the index by exact header-stamp ns). **Peak RSS on the
  real bag 416 MB → 57 MB.** Sample dtype confirmed **uint16** (2035 samples, beam 1).
  Down-channel fallback altitude is computed transiently during the scan (no samples
  stored). Caveat: window scan pads ±3 s for recv-vs-header skew (fine for lake latency;
  deep-water >3 s would need a larger pad). Per-scrub bag read is synchronous on the UI
  thread (small window, ~capped 600 pings) — async if it ever feels laggy.

## Estimated Scope

Multiple PRs (stacked). PR1 = scaffold + ingest + projection (core, tested). PR2 = map
canvas + swath/coverage render + scrub. PR3 = contact marking + store + cross-pass
overlay. Echogram / store-swap / bathy-altitude = later PRs.
