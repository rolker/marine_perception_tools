# Plan: CUBE lab: carry SonarDetections through cube::DetectionsProjector — real projector and error model instead of the angle-aware placeholder

## Issue

https://github.com/rolker/marine_perception_tools/issues/55

## Context

The CUBE lab currently projects M3 `SonarDetections` and weights soundings with
two homegrown placeholders: `src/mbes_geometry.hpp` (`project_beam`/
`project_detections`, a one-sound-speed re-derivation of `cube::Sounding`'s
geometry) and `src/sounding_uncertainty.hpp` (an angle-aware TPU stand-in whose
constants — `kRangeErrorPercent=0.005`, `kRangeErrorFloorM=0.05`,
`kAcrossTrackBeamwidthDeg=2.0` — were deliberately copied from `cube::Device`'s
own defaults). `cube_bathymetry` already ships the real thing:
`cube::DetectionsProjector::project(detections, tf, vessel_speed_mps)` returns
sonar-frame `cube::Sounding`s with real `vertical_error`/`horizontal_error`
(variances) from `cube::ErrorModel`, computed once per ping from TF-derived
roll/pitch/heave. `cube_bathymetry` is already a `find_package`d dependency
(`CMakeLists.txt:22`) — no new dependency.

**Two independent call sites** project detections today, both feeding
`MbesSounding` into the rest of the pipeline via `tf_lift::lift_sounding_to_world()`:
`src/sidescan_bag_session.cpp:972` (`SidescanBagSession::readMbesWindow`, the
3D point-cloud/backscatter-waterfall view) and `src/mbes_window_reader.cpp:202`
(`read_mbes_window`, used by `mbes_pass_loader.cpp` to gather the soundings
`cube_lab::run_cube` actually estimates from). Both need the swap.

**Design decisions settled here** (from the Issue Review):

1. **`mbes_geometry.hpp` is narrowed, not deleted.** It keeps `MbesSounding`
   (consumed by `point_cloud_view.hpp`, `mbes_pass_loader.hpp`,
   `mbes_window_reader.hpp`, `sidescan_bag_session.hpp`, `tf_lift.hpp`) and
   loses `project_beam`/`project_detections`. Its header comment is rewritten
   to say projection now happens via `cube::DetectionsProjector` elsewhere.
2. **Variance carrier: extend `MbesSounding`, don't add a parallel type.**
   `MbesSounding` already carries two CUBE-lab-only fields
   (`beam_angle`/`slant_range`) through the generic point-cloud pipeline that
   never reads them — `tf_lift.hpp`'s own doc comment says this is the
   intended extension point ("Any field added to `MbesSounding` rides along
   here for free, in both callers at once"). Adding `vertical_error` /
   `horizontal_error` (variances, m², NaN default, naming matches
   `cube::Sounding`) follows the established pattern instead of forking a
   second sounding type through `mbes_pass_loader`'s gather/box-clip path. The
   two fields ride unused through `point_cloud_view`/`mbes_window_reader` the
   same way `beam_angle`/`slant_range` already do.
3. **Offline config: library defaults, stated honestly.** No `cube::Vessel`/
   `cube::Device` config path exists offline (cube_bathymetry#145). The
   projector runs on `cube::Vessel{}`/`cube::Device{}` defaults (lever arms
   zero, generic device, M3 beamwidth = device fallback) and the lab's note
   says so, mirroring the placeholder's own honesty about skipped soundings.
4. **Vessel speed: NaN**, the same choice `cube_bathymetry`'s own offline
   tools make when no odometry is wired up. `cube::ErrorModel` accepts NaN and
   floors the speed-dependent horizontal term to 0. Wiring a real SOG source
   is not in this issue's Scope (items 1–4) or its Out of scope list — it is
   left as an explicit Open Question rather than silently added or silently
   skipped.
5. **TF frames need the `bizzy/` namespace**, like `SidescanBagOptions::base_frame`
   already does. `ProjectorParams` defaults to unprefixed `base_link`/
   `base_link_north_up`/`map_tide` (mru_transform's own unprefixed names,
   matching what `detections_to_pointcloud`'s live node overrides via ROS
   params). Whether `base_link_north_up`/`map_tide` actually exist as
   *namespaced* frames in the bags this lab reads offline is unverified here
   — if they don't, `DetectionsProjector` degrades gracefully (NaN roll/pitch,
   zero heave, both counted in diagnostics), which is exactly why item 4
   below surfaces those diagnostics rather than assuming success.

## Approach

1. **Add a shared offline-projection helper** — new header
   `src/mbes_projection.hpp` (+ `.cpp` since it links `cube_bathymetry`) —
   so the two call sites don't each hand-roll `ProjectorParams`/
   `DetectionsProjector` setup:
   - `offline_projector_params(base_link_frame, level_frame, tide_frame)` →
     `cube::ProjectorParams` with library-default `vessel`/`device`/range
     gates (decision 3).
   - `project_ping(const cube::DetectionsProjector &, detections, tf,
     vessel_speed_mps) -> {std::vector<MbesSounding>, cube::ProjectionDiagnostics}`,
     converting each `cube::Sounding` (`sonar_relative_position.{x,y,z}`,
     `intensity`, `beam_angle`, `slant_range`, `vertical_error`,
     `horizontal_error`) to an `MbesSounding`.
2. **Extend `MbesSounding`** (`mbes_geometry.hpp`) with `vertical_error` /
   `horizontal_error` (float, m², NaN default). Strip `project_beam`/
   `project_detections`; rewrite the file's header comment (decision 1).
3. **Delete `sounding_uncertainty.hpp`** outright (decision — issue scope #2).
4. **Rewire `SidescanBagSession`** (`sidescan_bag_session.{hpp,cpp}`): add
   `level_frame`/`tide_frame` to `SidescanBagOptions` (default
   `"bizzy/base_link_north_up"` / `"bizzy/map_tide"`, matching the existing
   `bizzy/`-prefixed `world_frame`/`base_frame` convention — decision 5); hold
   a `cube::DetectionsProjector` member built from `offline_projector_params`;
   in `readMbesWindow`, call `project_ping` instead of `project_detections`,
   still lifting with the unchanged `lift_sounding_to_world`. This path (the
   3D point-cloud/waterfall view) gets the real model for correctness parity
   with the CUBE lab; it has no existing per-window diagnostics UI surface, so
   this issue does not add one here (kept separate from item 6's note).
5. **Rewire `mbes_window_reader.cpp`**: same swap in `read_mbes_window`
   (`MbesWindowOptions` gains `level_frame`/`tide_frame`, same defaults);
   `MbesWindowResult` gains a `cube::ProjectionDiagnostics diagnostics` field,
   summed across the window's pings.
6. **Surface diagnostics in the lab's note** (issue scope #4):
   `mbes_pass_loader.cpp`'s `load_cloud_passes` already collects per-pass
   `notes` (`CloudLoadOutcome::notes`). Accumulate each pass's
   `MbesWindowResult::diagnostics` into a `cube::ProjectionRunTotals`
   (reusing the existing `cube_bathymetry/projection_summary.h` accumulator
   type and `report_projection_summary()` formatter instead of reinventing
   the text — `reports_georeferencing = false`, since this path's own
   "georeferenced" concept is the cross-bag earth-anchor reprojection, not
   per-sounding georeferencing) and append one summary line plus the
   library-defaults caveat (decision 3) to `notes` once per load.
7. **Rewrite `cube_lab.cpp`'s `run_cube`**: replace the
   `sounding_uncertainty(s.beam_angle, s.slant_range, &u)` computation
   (~line 315) with reading `s.vertical_error`/`s.horizontal_error` directly;
   drop a sounding (counted, like today) when either is non-finite or
   negative — same "no fabricated confidence" contract the placeholder used.
   Update the surrounding comment block (~line 307) and `cube_lab.hpp`'s
   `run_cube` doc comment (~lines 175-186), which currently describes the
   angle-aware placeholder.
8. **Test disposition**:
   - Delete `test/test_mbes_geometry.cpp` outright — it tests only
     `project_beam`/`project_detections`, which are retired.
     `cube_bathymetry` already has its own `test_detections_projector.cpp`
     covering `DetectionsProjector`'s geometry (which `mbes_geometry.hpp`'s
     own comment says it mirrored "exactly"), so there is no formula this
     package needs to re-verify.
   - Delete `test/test_sounding_uncertainty.cpp` outright — same reasoning,
     the file under test is retired.
   - Update `test/test_cube_lab.cpp`'s synthetic-sounding helpers: they
     currently set `beam_angle`/`slant_range` and rely on `run_cube`
     internally computing the placeholder error from them. With `run_cube`
     reading `vertical_error`/`horizontal_error` directly, these tests must
     set those two fields explicitly. Where a test's assertion depends on the
     angle-weighting *shape* (nadir vs. outer-beam confidence), inline the
     same first-order propagation formula the test needs as a local test
     helper (the production formula it was checking no longer exists to call
     into) — same values, so the existing assertions' expected numbers do not
     change.
   - Remove `test_mbes_geometry`/`test_sounding_uncertainty` gtest
     registrations from `CMakeLists.txt` (~lines 206-215, 279-283).
9. **`.agents/README.md`**: update the `sounding_uncertainty.hpp` line
   (remove) and the `mbes_geometry.hpp` line (describe it as the shared
   `MbesSounding` type + world-lift companion, not "per-beam projection").

## Files to Change

| File | Change |
|------|--------|
| `src/mbes_projection.hpp` + `.cpp` (new) | Shared `offline_projector_params()` + `project_ping()` helpers |
| `src/mbes_geometry.hpp` | Strip `project_beam`/`project_detections`; add `vertical_error`/`horizontal_error` to `MbesSounding`; rewrite header comment |
| `src/sounding_uncertainty.hpp` | Delete |
| `src/sidescan_bag_session.{hpp,cpp}` | `SidescanBagOptions` gains `level_frame`/`tide_frame`; hold a `DetectionsProjector`; `readMbesWindow` uses `project_ping` |
| `src/mbes_window_reader.{hpp,cpp}` | `MbesWindowOptions` gains `level_frame`/`tide_frame`; `MbesWindowResult` gains `diagnostics`; `read_mbes_window` uses `project_ping` |
| `src/mbes_pass_loader.cpp` | Accumulate `ProjectionRunTotals` across passes; append summary + library-defaults caveat to `CloudLoadOutcome::notes` |
| `src/cube_lab.{hpp,cpp}` | `run_cube` reads `vertical_error`/`horizontal_error` instead of calling `sounding_uncertainty()`; doc comments updated |
| `test/test_mbes_geometry.cpp` | Delete |
| `test/test_sounding_uncertainty.cpp` | Delete |
| `test/test_cube_lab.cpp` | Synthetic soundings set `vertical_error`/`horizontal_error` directly (local helper formula replaces the deleted production one) |
| `CMakeLists.txt` | Remove the two deleted tests' gtest registrations; add `mbes_projection.cpp`/`.hpp` to the library + test include paths that need it |
| `.agents/README.md` | Update the two layout-table rows |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Diagnostics (missing attitude/heave, default-beamwidth, range-filtered beam counts) and the library-defaults caveat are surfaced in `CloudLoadOutcome::notes` (item 6), not silently absorbed |
| A change includes its consequences | Both call sites (`sidescan_bag_session.cpp`, `mbes_window_reader.cpp`), `cube_lab.cpp`'s consumer, retired-file tests, and `.agents/README.md` are all in scope, not just the primary CUBE-lab path |
| Only what's needed | No SOG source is added (decision 4); no new UI diagnostics surface added to the 3D point-cloud path (item 4), which has none today; `mbes_geometry.hpp` is narrowed, not renamed, avoiding a 5-file include churn for no behavioral gain |
| Test what breaks | Retired-file tests deleted with their subject rather than left orphaned; `test_cube_lab.cpp`'s weighting assertions kept alive with equivalent local formulas so the angle-weighting behavior stays covered |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0007 (intensity rides the hypothesis queue bound to depth) | Yes | `project_ping`'s conversion keeps `intensity` bound to the same sounding as `beam_angle`/`depth`, matching `cube::Sounding`'s existing `{intensity, beam_angle}` pairing — no change to how `cube_lab.cpp` feeds `cs.intensity`/`cs.beam_angle` into `cube::Node::insert` |
| ADR-0008 (ROS 2 conventions) | Marginal | Header/library-only change inside an existing package; no new package, node, or launch surface |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `MbesSounding`'s fields | `tf_lift::lift_sounding_to_world` (copy-then-overwrite already carries new fields for free — no code change needed, verified) | Yes — no-op confirmed, noted |
| `sounding_uncertainty.hpp` retirement | `.agents/README.md`, `test/test_sounding_uncertainty.cpp` | Yes |
| `mbes_geometry.hpp` narrowing | `.agents/README.md`, `test/test_mbes_geometry.cpp` | Yes |
| `run_cube`'s error source | `cube_lab.hpp`/`cube_lab.cpp` doc comments describing the placeholder | Yes |
| Two independent projection call sites | Both `sidescan_bag_session.cpp` and `mbes_window_reader.cpp` | Yes — the Issue Review's variance-carrier finding surfaced only one; both were found during exploration |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.agents/README.md`'s layout-table
  rows for `sounding_uncertainty.hpp` (removed) and `mbes_geometry.hpp`
  (re-described); `cube_lab.hpp`/`cube_lab.cpp`'s doc comments describing the
  angle-aware placeholder.
- **Agent-instruction candidates**: None — this is a package-internal wiring
  change with no new workspace-wide pattern or pitfall to record.

## Open Questions

- [ ] Vessel speed-over-ground is passed as NaN (no odometry source wired
  into the CUBE lab offline). Should a follow-up issue wire a real SOG source
  (mirroring `cube_bathymetry import_bag_main`'s `--odom-topic`), given the
  error model floors the speed-dependent horizontal term to 0 without it? Not
  in this issue's stated scope.
- [ ] `level_frame`/`tide_frame` default to `bizzy/base_link_north_up` /
  `bizzy/map_tide` by convention with `base_frame`. Whether these frames are
  actually broadcast (namespaced) in the bags this lab reads is unverified
  without inspecting a real bag — if absent, the diagnostics (item 6) will
  show 100% `missing_attitude`/`missing_heave` and the note will say so; is
  that graceful-degradation-plus-note enough, or should the plan verify frame
  names against a real bag before implementation starts?

## Estimated Scope

Single PR — the two projection call sites, the shared helper, the estimator's
consumer, and the two retired-file cleanups are all one coherent swap; none of
it stands alone as a useful intermediate state.
