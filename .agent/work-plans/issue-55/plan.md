# Plan: CUBE lab: carry SonarDetections through cube::DetectionsProjector — real projector and error model instead of the angle-aware placeholder

## Issue

https://github.com/rolker/marine_perception_tools/issues/55

## Context

**Revision 4** (after the third Plan Review: 2 mechanical must-fixes — drop
invalid beams by value, not index; the cube_bathymetry link is per-target —
and 4 suggestions, all folded in below; ship recommended at that round).
Revision 3 (after the second Plan Review, changes-requested: 4 must-fix,
3 suggestions) is described next. Revision 2 (after the first Plan Review)
narrowed scope and settled the design decisions numbered 1–6 below, dated
2026-09-11 and owed to the operator directly, not inferred. Revision 3 adds
decision 7 (2026-09-14, operator: proceed on library defaults, park the
general fix as unh_marine_autonomy#385), the per-beam validity guard the
0.05 m gate did not replace, the warnings stream, and tests for the new
helper.

The CUBE lab currently weights soundings with a homegrown placeholder,
`src/sounding_uncertainty.hpp` (an angle-aware TPU stand-in whose constants —
`kRangeErrorPercent=0.005`, `kRangeErrorFloorM=0.05`,
`kAcrossTrackBeamwidthDeg=2.0` — were deliberately copied from
`cube::Device`'s own defaults), fed by `src/mbes_geometry.hpp`'s
`project_beam`/`project_detections` (a one-sound-speed re-derivation of
`cube::Sounding`'s geometry). `cube_bathymetry` already ships the real thing:
`cube::DetectionsProjector::project(detections, tf, vessel_speed_mps)` returns
sonar-frame `cube::Sounding`s with real `vertical_error`/`horizontal_error`
(variances) from `cube::ErrorModel`, computed once per ping from TF-derived
roll/pitch/heave. `cube_bathymetry` is already a `find_package`d dependency
(`CMakeLists.txt:22`) — no new dependency.

**Scope, narrowed (operator decision, 2026-09-11 — supersedes the "two
independent call sites" framing in revision 1):** this issue changes only the
CUBE-lab **estimator** path: `mbes_window_reader` → `mbes_pass_loader` →
`cube_lab::run_cube`. `SidescanBagSession::readMbesWindow` (the 3D
point-cloud/backscatter-waterfall view fed by a single bag's own scrub
window) and its placeholder projection are untouched here — that path is
[#56](https://github.com/rolker/marine_perception_tools/issues/56).
Consequence: `mbes_geometry.hpp`'s `project_beam`/`project_detections` keep
their one remaining caller (`SidescanBagSession::readMbesWindow`) and are
**not** removed in this issue; #56 removes them once that path is swapped
too.

**A wrinkle worth stating plainly, found while re-scoping this**:
`mbes_pass_loader`'s `load_cloud_passes()` — the one caller this issue does
change — feeds **both** `cube_lab::run_cube` (the CUBE estimate) **and** the
region-selection 3D point cloud (`sidescan_viewer_window.cpp:4228`,
`cloud_->setMultiPassPoints(out.pass_clouds)`). So after this issue, the
multi-pass selection cloud's `MbesSounding`s *will* carry real, non-fabricated
`vertical_variance`/`horizontal_variance` — it is not true that "the cloud
view keeps its placeholder projection" without qualification. What stays true,
and is the operator's actual scope line: the point-cloud *Uncertainty colour
channel* (`color_vocabulary.hpp`) is not wired to read them in this issue —
that UI exposure is deferred to #56 alongside the `SidescanBagSession` path,
so both cloud-facing surfaces move together. See must-fix 8 below.

**Design decisions settled here** (from the Issue Review, revision 1, and the
Plan Review's must-fixes):

1. **`mbes_geometry.hpp` is narrowed, not fully retired.** It keeps
   `MbesSounding` (consumed by `point_cloud_view.hpp`, `mbes_pass_loader.hpp`,
   `mbes_window_reader.hpp`, `sidescan_bag_session.hpp`,
   `sidescan_viewer_window.hpp` — 4 sites there, including `cube_soundings_` —
   `cube_lab.{hpp,cpp}`, and `tf_lift.hpp`) **and** `project_beam`/
   `project_detections` (still called from `SidescanBagSession::readMbesWindow`
   until #56). Its header comment is updated to say the CUBE-lab path now
   projects via `cube::DetectionsProjector` through the new
   `mbes_projection.hpp` helper, while this file's own projection functions
   remain the cloud path's until #56 retires them.
2. **Variance carrier: extend `MbesSounding`, don't add a parallel type** —
   unchanged from revision 1. `MbesSounding` already carries two
   CUBE-lab-only fields (`beam_angle`/`slant_range`) through the generic
   point-cloud pipeline that never reads them; `tf_lift.hpp`'s own doc
   comment says this is the intended extension point. The two new fields ride
   unused through `point_cloud_view`/`SidescanBagSession`'s cloud path the
   same way `beam_angle`/`slant_range` already do, until #56.
3. **New fields named `vertical_variance` / `horizontal_variance`, not
   `*_error`** (must-fix: naming). `cube::Sounding::vertical_error` /
   `horizontal_error` are themselves variances (m²) despite the name —
   [rolker/cube_bathymetry#158](https://github.com/rolker/cube_bathymetry/issues/158)
   is open to rename them, and it already counts sites "across cube and the
   explorer." These are brand-new fields with no compatibility burden, so
   this plan **leads** #158 rather than propagating its misnomer: the new
   `MbesSounding` fields are named for what they are. The
   `vertical_error`/`horizontal_error` → `vertical_variance`/
   `horizontal_variance` mapping happens exactly once, inside
   `project_ping()` in the new `mbes_projection.cpp` helper (Approach item
   1) — no other file translates the naming.
4. **Offline config: library defaults, stated honestly** — unchanged from
   revision 1. No `cube::Vessel`/`cube::Device` config path exists offline
   (cube_bathymetry#145). The projector runs on `cube::Vessel{}`/
   `cube::Device{}` defaults (lever arms zero, generic device, M3 beamwidth =
   device fallback) and both the CUBE-tuning dialog caveat and the lab's load
   note say so (Approach items 1, 3, 6).
5. **Vessel speed: NaN** — unchanged from revision 1, and verified
   (Plan Review, no change needed): `cube::ErrorModel::horizontal_latency`
   (`error_model.cpp:205-218`) floors a non-finite `platform.vessel_speed` to
   0 before squaring it into the speed-dependent horizontal terms
   (jitter/head/pitch latency error). A real SOG source is out of this
   issue's scope (still an Open Question below). **This makes the horizontal
   variance optimistic** — a real, moving vessel's latency-driven horizontal
   error is understated — and Approach item 1's shared caveat text says so
   explicitly, not just the NaN-input fact.
6. **TF frames: verified against a real bag, not trusted by convention**
   (must-fix, supersedes revision 1's "unverified" framing). The host read
   the TF tree from
   `/mnt/nadata/map2026asv/logs/gabby/logs/bizzy_timing/imu_sonar_timing_2026-08-20T16.55.09_postzda`
   (a BizzyBoat M3 bag, topic `/bizzy/sensors/m3/detections`, 2026-08-20).
   Verified transforms:
   - `/tf`: `earth → bizzy/map`, `bizzy/map → bizzy/base_link_north_up`,
     `bizzy/base_link_north_up → bizzy/base_link`,
     `bizzy/base_link_north_up → bizzy/base_link_level`,
     `bizzy/map → bizzy/odom`, `bizzy/odom → bizzy/map_tide`
   - `/tf_static`: `bizzy/base_link → bizzy/m3`,
     `bizzy/base_link → bizzy/deltat`, `bizzy/base_link → base_link` (a bare
     alias child), `base_link → base_link_frd`

   So the verified defaults for `MbesWindowOptions` (Approach item 5) are
   `base_link_frame = "bizzy/base_link"`,
   `level_frame = "bizzy/base_link_north_up"`,
   `tide_frame = "bizzy/map_tide"` — all namespaced, matching
   `SidescanBagOptions::base_frame`'s existing `bizzy/`-prefixed convention.
   A bare, unprefixed `base_link_frame = "base_link"` (the
   `cube::ProjectorParams` library default) would **not** have failed loudly:
   `/tf_static`'s `base_link → base_link_frd` alias chain means an unprefixed
   `base_link` resolves to *something*, silently, rather than throwing —
   which is exactly why must-fix 3 below (adding `base_link_frame` to
   `MbesWindowOptions` at all) matters: without it, `ProjectorParams` would
   default to unprefixed `base_link` while `level_frame`/`tide_frame` were
   namespaced, and the attitude lookup (`level_frame <- base_link_frame`)
   could never resolve to the real boat.
7. **Library-default vessel/device is what production already runs — say so,
   measure the cost, do not override** (must-fix: the real model changes the
   influence radius ~10×; operator decision 2026-09-14). `cube::Vessel{}`'s
   `gps_drms = 2.0` enters `swath_horizontal` as a constant
   `total_gps_variance = 4.0 m²` on every sounding (`error_model.cpp:105,395`),
   so `horizontal_error >= 4 m²` always and `Parameters::influenceRadius`
   caps spread at `CONF_99PC·sqrt(h) ≈ 5.2 m`, against the placeholder's
   `0.2 + 0.01·depth ≈ 0.5 m`. `run_cube`'s spread loop is
   O(radius²/cell²) per sounding, so up to ~100× more node inserts on a run
   already measured in minutes over ~1M soundings.

   Verified 2026-09-14: this is not a lab-only artefact. The live projector
   (`detections_to_pointcloud.cpp:128-137`) sets exactly three `Vessel`/
   `Device` fields from parameters (`ellipsoidal_referenced`,
   `range_error_percent`, `range_error_floor_m`) and BizzyBoat's config sets
   only the three frame names, so the boat, `import_bag`, `batch_regen_bag`,
   `bag_to_geotiff` and this lab all run the identical 2 m default. The lab
   matching them is not inventing a number; overriding `gps_drms` offline
   would invent one and make the lab disagree with the store. The general
   fix — a survey-configuration record the live node publishes and every
   offline tool reads — is parked for design as
   [unh_marine_autonomy#385](https://github.com/rolker/unh_marine_autonomy/issues/385)
   (with [cube_bathymetry#145](https://github.com/rolker/cube_bathymetry/issues/145)
   as its consumer half). Consequences in this plan:
   - `offline_projection_caveat()` names the **2 m generic-GPS assumption
     first**, before lever arms and the device fallback, and says the
     influence radius is therefore ~5 m (Approach item 1).
   - Approach item 12 adds a **runtime measurement** on a real pass so the
     cost is a number in the PR, not a prediction; a performance follow-up
     is filed only if that number hurts.

## Approach

1. **Add a shared offline-projection helper** — new header
   `src/mbes_projection.hpp` (+ `.cpp` since it links `cube_bathymetry`) —
   so `mbes_window_reader.cpp` doesn't hand-roll `ProjectorParams`/
   `DetectionsProjector` setup, and so the CUBE-tuning dialog and the load
   notes share one caveat string instead of two copies drifting apart:
   - `offline_projector_params(base_link_frame, level_frame, tide_frame)` →
     `cube::ProjectorParams` with library-default `vessel`/`device` (decisions
     4, 7) and **`minimum_range = kOfflineMinimumRangeM`**, a named constant
     in `mbes_projection.hpp` equal to `0.05` m (suggestion: one site to
     revisit, carrying the rationale) — not the library default `0.0`.
     Rationale: a beam whose range is exactly zero would otherwise sit at the
     sonar head with a spurious 0 m depth. 0.05 m is small enough that no
     genuine M3 return is filtered (its practical near-field range is tens of
     centimetres or more). Chosen to equal `cube::Device::range_error_floor_m`'s
     value only coincidentally — unrelated gates. Note (suggestion) that
     this is itself a small divergence from production: the live projector
     and the three offline tools run the library default `minimum_range =
     0.0`, so the lab filters a sliver of range they do not. Recorded here
     and in the constant's comment; #385's record is where the value should
     eventually live once, for all paths.

     **The range gate does not replace the guard `project_detections` had**
     (must-fix): that function skipped a beam when `twtt <= 0` **or**
     `sound_speed <= 0`, while cube's gate tests `range² ∈ [min², max²]`, so a
     *negative* travel time or sound speed produces a mirrored sounding whose
     squared range passes. `project_ping` therefore applies the same validity
     guard itself: if `ping_info.sound_speed <= 0` the whole ping is dropped
     (counted in a new `MbesWindowResult::invalid_pings`); an invalid beam
     is dropped **by value, not by index** — `DetectionsProjector::project`
     applies its range gate before returning (`detections_projector.cpp:200-209`),
     so sounding index ≠ beam index whenever any beam was filtered, which
     with `minimum_range > 0` and NaN `rx_angles` beams is the normal case.
     `cube::Sounding::slant_range` is `twtt · sound_speed / 2`
     (`sounding.h:45-49`), so `slant_range <= 0 || !std::isfinite(slant_range)`
     on the returned sounding is an exact, index-free equivalent of the
     retired `twtt <= 0` guard (given the ping-level `sound_speed > 0` check
     above). Dropped soundings are counted in `invalid_beams`. Both counts
     reach the load note (Approach item 7), whose arithmetic is then
     `beams = soundings + filtered_range + invalid_beams` per ping
     (suggestion: stated so a reader can check the note adds up).
   - `project_ping(const cube::DetectionsProjector &, detections, tf,
     vessel_speed_mps) -> {std::vector<MbesSounding>, cube::ProjectionDiagnostics}`,
     converting each `cube::Sounding` (`sonar_relative_position.{x,y,z}`,
     `intensity`, `beam_angle`, `slant_range`, and — decision 3 —
     `vertical_error`→`vertical_variance`, `horizontal_error`→
     `horizontal_variance`) to an `MbesSounding`.
   - `offline_projection_caveat()`: the one line of operator-facing text
     stating the library-defaults condition in order of consequence
     (decision 7 first: a generic 2 m GPS assumption on every sounding, so
     the influence radius is ~5 m and the same as the live store's; then
     decision 4: lever arms zero, generic device, M3 beamwidth = device
     fallback) **and** the NaN-speed consequence (decision 5: horizontal
     error is optimistic because the speed-dependent latency terms are
     floored to zero), pointing at unh_marine_autonomy#385 for the fix. Consumed by both
     the CUBE-tuning dialog (Approach item 4) and the lab's load note
     (Approach item 6) so there is exactly one place this wording lives.
2. **Extend `MbesSounding`** (`mbes_geometry.hpp`) with `vertical_variance` /
   `horizontal_variance` (float, m², NaN default, decision 3). Do **not**
   strip `project_beam`/`project_detections` (decision 1 — the cloud path
   still calls them); rewrite the file's header comment to describe the
   split ownership and point to #56.
3. **Delete `sounding_uncertainty.hpp`** outright. Its only remaining
   consumers after this issue are `cube_lab.cpp` (rewired below) and
   `sidescan_viewer_window.cpp`'s CUBE-tuning dialog (next item) — neither
   keeps a dependency on it.
4. **Rewire the CUBE-tuning dialog's caveat** (must-fix:
   `sidescan_viewer_window.cpp:112,934`). Line 112's
   `#include "sounding_uncertainty.hpp"` is replaced with
   `#include "mbes_projection.hpp"` (the dialog's translation unit does not
   otherwise need `mbes_geometry.hpp`'s projection functions — it already
   gets `MbesSounding` transitively via `cube_lab.hpp`, included from
   `sidescan_viewer_window.hpp:34`). Line 934's
   `sounding_uncertainty_caveat()` call becomes
   `offline_projection_caveat()` (Approach item 1) — same dialog placement
   (#45: the caveat belongs where the numbers are set), new text reflecting
   that the dialog now tunes the **real** `cube::ErrorModel`, running on
   library defaults, not a synthetic angle-only stand-in.
5. **Rewire `mbes_window_reader.cpp`**: `MbesWindowOptions` gains
   `base_link_frame`, `level_frame`, `tide_frame` (must-fix 3; verified
   defaults per decision 6 — `"bizzy/base_link"` /
   `"bizzy/base_link_north_up"` / `"bizzy/map_tide"`); `read_mbes_window`
   builds a `cube::DetectionsProjector` from `offline_projector_params` and
   calls `project_ping` per detections message instead of
   `project_detections`, still lifting with the unchanged
   `lift_sounding_to_world`. Vessel speed is passed as NaN (decision 5) —
   `read_mbes_window` has no odometry source, same as today.
6. **`MbesWindowResult` gains `cube::ProjectionRunTotals diagnostics`**
   (must-fix: `ProjectionDiagnostics` alone carries no ping/beam/sounding
   counts, so a summed-`ProjectionDiagnostics` field cannot drive
   `report_projection_summary`'s "N soundings" line or its zero-sounding
   warning — `ProjectionRunTotals` is the type that has both the counts and
   the accumulator semantics `cube_bathymetry`'s own offline tools already
   use it for). `read_mbes_window` accumulates one ping at a time: `pings`
   +=1, `beams` += the ping's `ProjectionDiagnostics::total` (suggestion:
   `ErrorModel::compute` emits exactly one sounding per beam and never
   skips, so the denominator cannot drift from the numerators), `soundings`
   += kept soundings,
   `filtered_range`/`missing_attitude`/`missing_heave`/
   `default_beamwidth_beams`/`missing_rx_angle_beams` += the per-ping
   `ProjectionDiagnostics`' matching fields, `reports_georeferencing = false`
   (this path's own "georeferenced" concept — `MbesWindowResult::has_geo` /
   `earth_from_world`, the cross-bag earth-anchor reprojection — is a
   different thing from per-sounding georeferencing, and `report_projection_summary`
   must not conflate the two).
7. **Surface diagnostics + the caveat in the lab's note** (must-fix: honest
   degradation). `mbes_pass_loader.cpp`'s `load_cloud_passes` accumulates
   each pass's `MbesWindowResult::diagnostics` (item 6) field-by-field into a
   running `cube::ProjectionRunTotals` across the whole load. After the
   loop, if `totals.pings > 0`, call
   `cube_bathymetry/projection_summary.h`'s `report_projection_summary()`
   into two `ostringstream`s (reusing the existing formatter instead of
   reinventing the text — the summary line always includes the
   `missing_attitude`/`missing_heave` counts, not just the warning paths, so
   a frame mismatch is visible even when it does not zero out the sounding
   count) and append **both streams** — the summary line **and every line of
   the `err` stream** (must-fix: that is where the default-beamwidth warning
   lives, and `kongsberg_em_bridge` leaves `rx_beamwidths` empty on every M3
   ping, so it fires on 100% of beams in exactly this deployment; discarding
   it would hide the one warning this deployment always produces) — plus
   `offline_projection_caveat()` (Approach item 1) to
   `CloudLoadOutcome::notes`, once per load. Alongside, one line per load
   for the drop populations `ProjectionRunTotals` has no field for
   (suggestion): `read_mbes_window`'s existing `skipped_pings` (ping dropped
   for a missing world←sensor TF before projection) and the new
   `invalid_pings`/`invalid_beams` from Approach item 1, summed across
   passes the same way — so every place a sounding can vanish is visible in
   the same note.

   **Why this alone is not sufficient** — the honest-degradation gap the review
   found: missing attitude makes `vertical_error`/`horizontal_error` NaN but
   leaves the *position* finite, so `minimum_range`'s range gate (item 1)
   keeps the sounding; `report_projection_summary`'s own frame-mismatch
   warning only fires when `totals.soundings == 0`, which does not happen
   here. Without item 8 below, the only visible symptom would be
   `run_cube`'s existing drop count — mislabelled "no beam geometry" when
   the actual cause is an unresolved attitude TF, i.e. exactly the frame
   mismatch this note exists to catch.
8. **Rewrite `cube_lab.cpp`'s `run_cube`**: replace the
   `sounding_uncertainty(s.beam_angle, s.slant_range, &u)` computation
   (~line 315) with reading `s.vertical_variance`/`s.horizontal_variance`
   directly into `cs.vertical_error`/`cs.horizontal_error` (the field names
   `cube::Sounding` itself uses — decision 3's rename applies only to
   `MbesSounding`, not to the library type). Drop a sounding when
   `params.influenceRadius(cs)` comes back non-finite, **reusing
   `Parameters::influenceRadius`'s own gate** (must-fix/suggestion:
   `influenceRadius` already rejects `vertical_error <= 0.0` — not just
   negative — and `horizontal_error < 0.0`, non-finite depth, or non-finite
   either variance, returning NaN; today's hand-rolled
   `sounding_uncertainty()` check only tested non-finite/negative, which
   would let a **zero** vertical variance through to a NaN-radius cast to
   `int` for the spread loop's bounds. Calling `influenceRadius` once and
   branching on `std::isfinite(radius)` replaces the duplicated gate with
   the single one CUBE itself enforces, instead of keeping two gates that
   can drift apart). Reword the skip note (must-fix: attribution) from
   `"N sounding(s) skipped — no beam geometry"` to
   `"N sounding(s) skipped — invalid uncertainty (see load notes for missing-attitude/heave counts)"`
   — the drop is no longer specifically about beam geometry (the real error
   model can also NaN a sounding via missing attitude/heave), and the
   reworded text points the operator at item 7's note instead of guessing.
   Update the surrounding comment block (~line 307) and `cube_lab.hpp`'s
   `run_cube` doc comment (~lines 175-186): drop the `#49`/
   `sounding_uncertainty.hpp` description, describe the real
   `cube::ErrorModel` via `mbes_projection.hpp`, state the library-defaults
   condition and the NaN-speed → optimistic-horizontal-error consequence
   (decisions 4, 5) in one line each, and note that `horizontal_variance` is
   radial/drms and feeds `influenceRadius` directly, so a sounding's spread
   — and the surface's visual character — changes under the real model
   (suggestion).
9. **Test disposition**:
   - **Keep** `test/test_mbes_geometry.cpp` (revision 1 said delete this —
     wrong, since decision 1 now keeps `project_beam`/`project_detections`
     alive for the cloud path until #56). No change needed to the file
     itself.
   - Delete `test/test_sounding_uncertainty.cpp` outright — the file under
     test is retired and has no other consumer.
   - Update `test/test_cube_lab.cpp`'s synthetic-sounding helpers: they
     currently set `beam_angle`/`slant_range` and rely on `run_cube`
     internally computing the placeholder error from them. With `run_cube`
     reading `vertical_variance`/`horizontal_variance` directly, these tests
     must set those two fields explicitly. Where a test's assertion depends
     on the angle-weighting *shape* (the `TwoPassSurfaceOverACleanAndANoisyPass`
     characterisation test, ~line 549, and any other nadir-vs-outer-beam
     assertion), inline the same first-order propagation formula the test
     needs as a **local test-only helper** (the production formula it was
     checking no longer exists to call into) — same values, so the existing
     assertions' expected numbers do not change. Update the test's own
     comment (~line 545, "unit-tested in test_sounding_uncertainty") to
     point at the local helper instead of the deleted file.
   - Extend `test/test_tf_lift.cpp`'s (suggestion) NaN carry-through test
     (`TfLift.UnknownGeometryStaysUnknown`) and the rotated-lift test
     (`TfLift.BeamGeometryAndIntensitySurviveARotatedTranslatedLift`) to also
     cover `vertical_variance`/`horizontal_variance`, instead of only
     asserting the "any field rides along" property in prose (`tf_lift.hpp`'s
     own doc comment).
   - Remove only `test_sounding_uncertainty`'s gtest registration from
     `CMakeLists.txt` (~lines 279-283). `test_mbes_geometry`'s registration
     (~lines 206-210) stays.
10. **`color_vocabulary.hpp` / `test_color_vocabulary.cpp`** (must-fix: the
    verified-gap text becomes false). `point_channel_unavailable_reason`'s
    `ColorChannel::Uncertainty` case (`color_vocabulary.hpp:65-80`) currently
    says "the per-beam errors CUBE uses are a placeholder computed inside
    the estimator from the beam's angle and slant range" — after this issue
    that is no longer true for the CUBE-lab path (it is real
    `cube::ErrorModel` output). Reworded text: the point cloud's
    `MbesSounding`s carry real `vertical_variance`/`horizontal_variance` as
    of this issue, but the point-cloud **colour channel** for Uncertainty
    stays unavailable here — deliberately deferred to #56, so both
    cloud-facing surfaces (`SidescanBagSession`'s single-window cloud and
    the colour-channel wiring) move together rather than exposing the
    channel for one cloud source (`mbes_pass_loader`'s multi-pass selection)
    and not the other (`SidescanBagSession::readMbesWindow`'s scrub-window
    cloud), which would make the same channel silently mean different things
    depending on which cloud is on screen. `test_color_vocabulary.cpp:73-88`'s
    comment and assertions are updated to match: the test still expects
    `Uncertainty` greyed for points, but the reason text and the "verified
    gap" framing change to "real variances exist on some cloud sources now;
    the channel itself opens in #56 once both sources carry them."
11. **`.agents/README.md`**: update the `sounding_uncertainty.hpp` line
    (remove), the `mbes_geometry.hpp` line (describe it as the shared
    `MbesSounding` type + the cloud path's own projection, not "per-beam
    projection" generally — the CUBE-lab path now projects elsewhere), and
    add a new row for `mbes_projection.{hpp,cpp}` (suggestion — the CUBE-lab
    path's real projection + error-model wiring, offline defaults, and the
    shared caveat text).
12. **Measure the runtime cost once, on a real pass** (decision 7). Before
    and after the swap, time `run_cube` over the same multi-pass selection
    from an archived BizzyBoat M3 bag (the 2026-08-20 `bizzy_timing` bag
    used for decision 6, or a `bizzyboat_sonar` recording) at the lab's
    default cell size. **How** (suggestion: `run_cube` has no headless
    caller outside `test_cube_lab`): drive it through the explorer itself —
    load the passes in the survey explorer, run the CUBE lab, and read the
    elapsed time from the lab's existing load/run notes (adding a wall-time
    line to `CloudLoadOutcome::notes` / the CUBE run note if none exists,
    which is a one-line `std::chrono` addition and stays in). Record wall
    time, sounding count and node-insert count (if `run_cube` exposes one;
    otherwise wall time and soundings) in `progress.md`'s Implementation
    entry and the PR body. A performance
    follow-up is filed only if the measured slowdown makes the lab
    unusable, with the number in it — not on the ~100× prediction alone.
13. **Tests for the new helper and the two changed readers** (must-fix):
    - New `test/test_mbes_projection.cpp` (registered in `CMakeLists.txt`,
      linking `cube_bathymetry`): `offline_projector_params()` returns the
      three frame names it was given, `minimum_range == kOfflineMinimumRangeM`,
      default-constructed `vessel`/`device`; `project_ping()` on a synthetic
      `SonarDetections` + identity TF maps `sonar_relative_position`,
      `intensity`, `beam_angle`, `slant_range` and — the one rename site —
      `vertical_error`→`vertical_variance`, `horizontal_error`→
      `horizontal_variance` per beam; a ping with `sound_speed <= 0` yields
      no soundings and `invalid_pings == 1`; a beam with `twtt <= 0` (zero
      **and** negative) is dropped with `invalid_beams` counted while its
      siblings survive; `ProjectionDiagnostics` passes through unchanged.
    - `test/test_mbes_window_reader.cpp`: `MbesWindowOptions` defaults equal
      the verified `bizzy/` frames (decision 6); `MbesWindowResult`'s
      `diagnostics` accumulates `pings`/`beams`/`soundings` correctly across
      two synthetic pings (unit-level, feeding `read_mbes_window`'s
      accumulation step directly if the function is split to allow it, else
      via the smallest in-memory bag fixture the existing tests use).
    - `test/test_mbes_pass_loader.cpp`: `CloudLoadOutcome::notes` contains the
      summary line, the `err`-stream warning lines, the skipped/invalid line
      and `offline_projection_caveat()` exactly once per load.

## Files to Change

| File | Change |
|------|--------|
| `src/mbes_projection.hpp` + `.cpp` (new) | `kOfflineMinimumRangeM`; `offline_projector_params()` (library-default vessel/device, `minimum_range = kOfflineMinimumRangeM`); `project_ping()` (maps `cube::Sounding::{vertical,horizontal}_error` → `MbesSounding::{vertical,horizontal}_variance`, drops `sound_speed <= 0` pings and `twtt <= 0` beams with counts); `offline_projection_caveat()` shared text, 2 m GPS first |
| `test/test_mbes_projection.cpp` (new) | Defaults, constant, rename mapping, validity guard (zero and negative), diagnostics passthrough |
| `test/test_mbes_window_reader.cpp` | Verified frame defaults; `ProjectionRunTotals` accumulation across pings |
| `test/test_mbes_pass_loader.cpp` | Load note carries summary + `err` warnings + skipped/invalid line + caveat once |
| `src/mbes_geometry.hpp` | Add `vertical_variance`/`horizontal_variance` to `MbesSounding`; **keep** `project_beam`/`project_detections` (cloud path, until #56); rewrite header comment for the split |
| `src/sounding_uncertainty.hpp` | Delete |
| `src/sidescan_viewer_window.cpp` | Line 112: `#include "mbes_projection.hpp"` in place of `sounding_uncertainty.hpp`; line 934: `offline_projection_caveat()` in place of `sounding_uncertainty_caveat()` |
| `src/mbes_window_reader.{hpp,cpp}` | `MbesWindowOptions` gains `base_link_frame`/`level_frame`/`tide_frame` (verified defaults); `MbesWindowResult` gains `cube::ProjectionRunTotals diagnostics` + `invalid_pings`/`invalid_beams`; `read_mbes_window` holds a `DetectionsProjector`, calls `project_ping`, accumulates totals with `beams` from `ProjectionDiagnostics::total` |
| `src/mbes_pass_loader.cpp` | Accumulate `MbesWindowResult::diagnostics` (+ `skipped_pings`, `invalid_pings`, `invalid_beams`) across passes; append `report_projection_summary()`'s summary **and `err` warnings**, the skipped/invalid line, and `offline_projection_caveat()` to `CloudLoadOutcome::notes` once per load |
| `src/cube_lab.{hpp,cpp}` | `run_cube` reads `vertical_variance`/`horizontal_variance` into `cs.vertical_error`/`horizontal_error`; drop gate reuses `params.influenceRadius(cs)`'s own NaN result; skip note reworded (attribution); doc comments updated (real model, library defaults, NaN-speed caveat, `horizontal_variance` feeds `influenceRadius`) |
| `src/color_vocabulary.hpp` | Reword `ColorChannel::Uncertainty`'s point-unavailable reason (real variances now exist on some sources; channel wiring deferred to #56) |
| `test/test_color_vocabulary.cpp` | Update comments/assertions to match the reworded reason text |
| `test/test_sounding_uncertainty.cpp` | Delete |
| `test/test_mbes_geometry.cpp` | **No change** — stays; still tests `project_beam`/`project_detections`, which stay until #56 |
| `test/test_cube_lab.cpp` | Synthetic soundings set `vertical_variance`/`horizontal_variance` directly (local test-only helper formula replaces the deleted production one); update the stale `test_sounding_uncertainty` comment reference |
| `test/test_tf_lift.cpp` | Extend `sampleSounding()` and the carry-through/rotated-lift assertions **and `IdentityTransformCarriesEveryField` (line 52)** to cover the two new fields |
| `CMakeLists.txt` | Remove only `test_sounding_uncertainty`'s gtest registration; register the new `test_mbes_projection`. **Per-target consequence** (must-fix, round 3): `mbes_window_reader.cpp` lives in `sidescan_core`, whose `SIDESCAN_CORE_DEPS` (line 107) has no `cube_bathymetry` and whose consumers get no `CUBE_BATHYMETRY_INCLUDE_ROOT` SYSTEM include — "no new dependency" is true per-package, false per-target. Adding `mbes_projection.cpp` to `sidescan_core` and `cube::ProjectionRunTotals` to the public `MbesWindowResult` means: add `cube_bathymetry` to `SIDESCAN_CORE_DEPS` and the SYSTEM include to `sidescan_core`'s PUBLIC include dirs, so it propagates to every `sidescan_core` consumer — `sidescan_probe` (128-129), `test_mbes_window_reader`, `test_mbes_pass_loader`, `test_session_index_io` (294-295), `test_cube_lab` — rather than patching each target |
| `.agents/README.md` | Update `sounding_uncertainty.hpp` (remove) and `mbes_geometry.hpp` (re-describe) rows; add a `mbes_projection.{hpp,cpp}` row |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Human control and transparency | Diagnostics (missing attitude/heave, default-beamwidth, range-filtered beam counts) and the library-defaults + NaN-speed caveat are surfaced in `CloudLoadOutcome::notes` and the CUBE-tuning dialog (Approach items 4, 6, 7), not silently absorbed; the "no beam geometry" skip note is reworded so its cause is correctly attributed instead of misleading (Approach item 8) |
| A change includes its consequences | The one in-scope call site (`mbes_window_reader.cpp` via `mbes_pass_loader.cpp`), its `cube_lab.cpp` consumer, the CUBE-tuning dialog's caveat, `color_vocabulary.hpp`'s now-stale verified-gap text, retired-file tests, and `.agents/README.md` are all in scope — not just the primary estimator path |
| Only what's needed | No SOG source is added (decision 5); no new UI diagnostics surface added to the cloud path, which is explicitly out of scope this issue (#56); `mbes_geometry.hpp` keeps its projection functions rather than deleting and re-adding them next issue |
| Test what breaks | `test_sounding_uncertainty.cpp` deleted with its subject; `test_mbes_geometry.cpp` correctly kept alive since its subject survives; `test_cube_lab.cpp`'s weighting assertions kept alive with an equivalent local formula; `test_tf_lift.cpp` extended to the two new fields instead of asserting the carry-through property in prose only; the new helper gets its own `test_mbes_projection.cpp` (rename mapping, validity guard incl. negative inputs) and the two readers whose types change get tests for the new totals and notes (Approach item 13) |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0007 (intensity rides the hypothesis queue bound to depth) | Yes | `project_ping`'s conversion keeps `intensity` bound to the same sounding as `beam_angle`/`depth`, matching `cube::Sounding`'s existing `{intensity, beam_angle}` pairing — no change to how `cube_lab.cpp` feeds `cs.intensity`/`cs.beam_angle` into `cube::Node::insert` |
| ADR-0008 (ROS 2 conventions) | Marginal | Header/library-only change inside an existing package; no new package, node, or launch surface |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `MbesSounding`'s fields | `tf_lift::lift_sounding_to_world` (copy-then-overwrite already carries new fields for free — no code change needed, verified); `test_tf_lift.cpp`'s carry-through assertions | Yes |
| `sounding_uncertainty.hpp` retirement | `.agents/README.md`, `test/test_sounding_uncertainty.cpp`, `sidescan_viewer_window.cpp`'s CUBE-tuning dialog (its only other consumer) | Yes — the Plan Review caught the dialog consumer that revision 1 missed |
| `run_cube`'s error source | `cube_lab.hpp`/`cube_lab.cpp` doc comments describing the placeholder; `color_vocabulary.hpp`'s verified-gap text; `test_color_vocabulary.cpp` | Yes |
| `MbesWindowResult` gains real diagnostics totals | `mbes_pass_loader.cpp`'s note accumulation; `cube_lab.cpp`'s skip-note wording, which otherwise still mislabels a frame-mismatch drop as "no beam geometry" | Yes |
| Scope narrowed to one call site | `mbes_geometry.hpp`'s `project_beam`/`project_detections` and `test_mbes_geometry.cpp` are correctly **not** touched this issue | Yes — revision 1 got this backwards |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.agents/README.md`'s layout-table
  rows for `sounding_uncertainty.hpp` (removed), `mbes_geometry.hpp`
  (re-described), and a new `mbes_projection.{hpp,cpp}` row;
  `cube_lab.hpp`/`cube_lab.cpp`'s doc comments describing the angle-aware
  placeholder; `color_vocabulary.hpp`'s `ColorChannel::Uncertainty`
  verified-gap comment and `test_color_vocabulary.cpp`'s matching comment.
- **Agent-instruction candidates**: None — this is a package-internal wiring
  change with no new workspace-wide pattern or pitfall to record.

## Open Questions

- [ ] Vessel speed-over-ground is passed as NaN (no odometry source wired
  into the CUBE lab offline). Should a follow-up issue wire a real SOG source
  (mirroring `cube_bathymetry import_bag_main`'s `--odom-topic`), given the
  error model floors the speed-dependent horizontal term to 0 without it,
  making horizontal error optimistic? Not in this issue's stated scope.
- [ ] `kOfflineMinimumRangeM = 0.05 m` (Approach item 1) is a plan-time
  choice, not a value verified against the M3's actual near-field spec sheet.
  If a real M3 bag is later found to report legitimate detections inside
  5 cm, the one constant needs revisiting — flagged here rather than treated
  as settled physics.
- [x] The 2 m generic-GPS default (decision 7): **resolved for this issue** —
  proceed on library defaults, matching production; the general fix is
  unh_marine_autonomy#385. Left here so the item stays visible in review.

## Estimated Scope

Single PR — the shared helper, the one in-scope call site
(`mbes_window_reader.cpp` via `mbes_pass_loader.cpp`), the estimator's
consumer (`cube_lab.cpp`), the CUBE-tuning dialog's caveat, the
`color_vocabulary.hpp` text update, and the retired-file cleanup are all one
coherent swap; none of it stands alone as a useful intermediate state. The
cloud path (`SidescanBagSession::readMbesWindow`, `mbes_geometry.hpp`'s
projection functions, and the Uncertainty colour channel) is explicitly
deferred to #56.
