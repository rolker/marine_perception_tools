# Plan — marine_perception_tools #8: MBES 3D cloud + backscatter waterfall + water-column echogram

**Issue:** [#8](https://github.com/rolker/marine_perception_tools/issues/8) — extend the offline
sidescan viewer (#7, merged) with MBES displays so an analyst sees map + both
waterfalls + 3D point cloud + water-column echogram at once, driven by one
distance scrub. Blockers cleared: #7 merged; `marine_sonar_widgets#1` widgets on
jazzy + in the manifest.

**Approach:** one feature branch (`feature/issue-8`), staged commits (mirrors #7's
PR1/PR2/PR3 rhythm), one PR at the end. Each stage builds + tests green before the
next. Reuse the shared widgets; the only genuinely new GUI piece is the 3D viewport.

**Resequenced 2026-06-24 (Roland, mid-ops): MBES value first.** Stage 1 (retire
`SidescanWaterfall`) is a pure refactor with no new visible value, so it was
deferred and built **last**, after the MBES panes delivered their value — the
msw#6 overlay merge removed its only blocker, so the MBES panes were built on the
lib widget without retiring `SidescanWaterfall` first. Order as executed: **Stage 0
(lib dep) → 2 (MBES core) → 3 (backscatter) → 4 (3D cloud) → 5 (echogram) →
6 (docks) → 1 (retire).** All stages complete as of 2026-06-24; the sidescan pane
now also uses the lib `WaterfallWidget` (Stage 1), so all four sonar panes share it.

## Ground truth (verified, not assumed)

- **Detections** = `marine_acoustic_msgs/msg/SonarDetections` on
  `/bizzy/sensors/m3/detections` (frame `bizzy/m3`, 224 beams). Per-beam fields:
  `two_way_travel_times[]`, `tx_angles[]`, `rx_angles[]`, `intensities[]` (dB);
  `ping_info.sound_speed`.
- **Beam geometry** (mirror `cube_bathymetry/include/cube_bathymetry/sounding.h`,
  sensor frame, single sound-speed, no CUBE error model):
  `range = twtt[i]·ss/2`; `x = range·-sin(tx[i])`; `y = range·sin(rx[i])`;
  `z = range·cos(tx[i])·cos(rx[i])`; `intensity = intensities[i]`.
- **Water column** = Garmin `sonar_image_down` (`RawSonarImage`) → `EchogramWidget::addPings`.
- **Existing seams:** `SidescanBagSession` (windowed read + pose/time table,
  `timeAtDistance`, `mapToGeo`); `SidescanViewerWindow` (async `SidescanRenderResult`
  with epoch stale-drop; `onScrubChanged`→`requestRender`; marking
  `onContactMarked(QRectF map)`→`make_box_contact`→`ContactStore`);
  `sidescan_geometry.hpp` (projection + WGS84↔ECEF).
- **Lib widgets:** `WaterfallWidget` (geometry-agnostic, `add_row(WaterfallRow)`,
  newest-at-top, GPU colormap, `setMarkMode`/`boxMarked(QRectF)`),
  `EchogramWidget` (`addPings(RawSonarImage)`).

## Stages

### Stage 0 — depend on the shared widgets, no behavior change
- `package.xml` + `CMakeLists.txt`: add `marine_sonar_widgets`.
- `.github/workflows/ci.yml`: clone source siblings the lib needs that aren't in
  rosdep — `marine_sonar_widgets` (jazzy) and `marine_colormap` (jazzy); the existing
  `marine_interfaces` clone-and-prune stays. (`marine_acoustic_msgs` via rosdep.)
- Build green; viewer unchanged. **Gate: `make build` + existing 206 tests pass.**

### Stage 1 — retire the bespoke `SidescanWaterfall`, use the lib `WaterfallWidget`
- Replace the CPU `SidescanWaterfall` QImage pane with the GPU `WaterfallWidget`,
  fed `WaterfallRow`s built from the windowed sidescan pings (port/stbd ranges,
  nadir, per-row `world_pose` from the pose table so the lib's pixel→map marking
  works). Delete `sidescan_waterfall.{hpp,cpp}` + `SidescanRenderResult.waterfall`
  /`waterfall_index` (the lib widget owns marking now).
- Re-wire marking: lib `boxMarked(QRectF map)` → existing `onContactMarked`.
- **Gate:** marking on the sidescan pane still produces the same `Contact`
  (map-frame + geo_pose); offscreen smoke passes.

### Stage 2 — MBES soundings core (Qt-free, tested)
- New `mbes_geometry.hpp` (header-only, like `sidescan_geometry.hpp`): `project_beam()`
  mirroring the cube formula; `project_detections(SonarDetections, sound_speed)` →
  `vector<Sounding{x,y,z,intensity}>` in sensor frame.
- `SidescanBagSession`: windowed read of `/bizzy/sensors/m3/detections`; transform
  sensor→world via static `m3→base` + per-ping `base→world` from the pose table
  (reuse the existing machinery); join onto the distance axis like the sidescan pings.
- **Gate:** gtests assert `project_beam` matches cube `Sounding` to 1e-9 on a
  synthetic ping; windowed read returns the expected ping/point counts.

### Stage 3 — MBES backscatter waterfall pane
- Second `WaterfallWidget`; each detection ping → one `WaterfallRow` of 224
  `intensities` (across-track = beam index; metric optional). Driven by the shared scrub.
- **Gate:** offscreen render of a synthetic detections window; rows newest-at-top.

### Stage 4 — 3D point cloud pane (the new viewport)
- New `point_cloud_view.{hpp,cpp}` — `QOpenGLWidget`, GeoZui-style orbit
  (left-drag = heading + pitch, scroll = zoom), Z-exaggeration slider, color by
  **depth** (default) or **backscatter**, view-only. Renders the windowed soundings
  (world frame, recentred on the scrub window centroid). Software-GL safe + self-skip
  offscreen tests, matching the lib widgets.
- **Gate:** offscreen smoke (context + draw, no crash); color-toggle + exaggeration
  change the buffer as expected.

### Stage 5 — water-column echogram pane
- `EchogramWidget` fed `sonar_image_down` `RawSonarImage` for the scrub window via
  `addPings`. Fast-follow per the issue.
- **Gate:** offscreen smoke.

### Stage 6 — dock layout
- Convert the panes to `QDockWidget`s — map + sidescan waterfall + MBES backscatter
  waterfall + 3D cloud + echogram, all visible, rearrangeable, floatable to a 2nd
  monitor. Persist layout via `QSettings`. One shared scrub/window-length/max-pings.
- **Gate:** all five panes render the same scrub window in sync (acceptance crit).

## Testing
- gtests: `project_beam`/`project_detections` vs cube convention; detections→`WaterfallRow`
  adapter; windowed read counts.
- Offscreen smoke (software GL, self-skip when unavailable) for each GL pane, like
  the lib widget tests.
- Manual: load a `bizzyboat_sonar` bag (`~/data/logs/gabby/logs/bizzyboat_sonar/<ts>/`);
  confirm all panes sync; orbit + exaggeration + color toggle; mark a target.

## Out of scope (v1, per issue)
3D marking/picking (view-only); CUBE-grade gridding/error model; full-survey
persistent grid (cloud = scrub window by design).

## Consumer-thinning note
Stage 1 retires the sidescan `SidescanWaterfall` (the waterfall half of
`marine_sonar_widgets#1` consumer thinning for this repo). The `make_box_contact`
half — swapping `contact_store.cpp`'s local copy for
`#include <marine_sonar_widgets/contact_builder.hpp>` + a `using` alias — is small
and can ride in Stage 0/1 or stay a separate follow-up; default: fold into Stage 1
so the duplication closes here. `msw#1` can then close.
