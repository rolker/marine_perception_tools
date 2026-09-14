---
issue: 55
---

# Issue #55 — CUBE lab: carry SonarDetections through cube::DetectionsProjector — real projector and error model instead of the angle-aware placeholder

## Issue Review
**Status**: complete
**When**: 2026-09-11 14:35 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Issue**: #55
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Item 2's file citation (`src/mbes_bag_session.hpp:59,341`) doesn't exist — the detections topic + `tf2::BufferCore` actually live in `src/sidescan_bag_session.hpp` (content matches the description; only the filename is stale). Correct the reference during plan-task.
- [ ] Item 2 says "Retire `src/mbes_geometry.hpp` ... outright" — this overstates the change. The file defines both the redundant `project_beam`/`project_detections` functions (the real duplicate of `DetectionsProjector`'s stage-2 math) **and** the `MbesSounding` struct, which is the general-purpose sounding type used well outside the CUBE lab: `point_cloud_view.hpp`, `mbes_pass_loader.hpp`, `mbes_window_reader.hpp`, `sidescan_bag_session.hpp`, and `tf_lift.hpp` all consume it for the 3D point-cloud path, none of which this issue's scope touches. Deleting the whole file breaks those. plan-task should scope precisely: retire the two projection functions; decide where `MbesSounding` (or its replacement) lives.
- [ ] Variance carry-through isn't addressed. `cube_lab.cpp` (`run_cube`, ~line 315) currently computes variance from `MbesSounding::beam_angle`/`slant_range` via `sounding_uncertainty()` at CUBE-insertion time — well after soundings are lifted to world frame and gathered/box-clipped across possibly many passes (`mbes_pass_loader` → `cube_lab`). `DetectionsProjector::project` instead computes `cube::Sounding`'s vertical/horizontal variance once, at ping-processing time (it needs the TF buffer + vessel speed, which are only in scope then). `tf_lift::lift_sounding_to_world()` — the helper Scope item 1 says to reuse — operates on `MbesSounding`, which has no variance field, so the real model's per-sounding variance has nowhere to ride between projection and CUBE insertion as things stand. plan-task should decide explicitly: extend `MbesSounding` with variance fields (which then flow, unused, through `point_cloud_view`/`mbes_pass_loader` too), or carry a CUBE-lab-specific sounding type alongside it.
- [ ] Scope doesn't mention `test/test_mbes_geometry.cpp` and `test/test_sounding_uncertainty.cpp`, the existing coverage for the two retired files. Per "Test what breaks," plan-task should decide their disposition (delete outright vs. migrate any formula-equivalence assertions into cube_bathymetry's own `DetectionsProjector` tests) rather than letting coverage silently vanish with the placeholder.
- [ ] `.agents/README.md`'s Repository Layout table describes `mbes_geometry.hpp` ("per-beam projection to the sensor frame") and `sounding_uncertainty.hpp` ("PLACEHOLDER per-sounding TPU") by their current, soon-outdated roles — update in the same PR ("a change includes its consequences").

## Plan Authored
**Status**: complete
**When**: 2026-09-11 14:43 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Plan**: `.agent/work-plans/issue-55/plan.md` at `c7b8bce`
**Branch**: feature/issue-55 at `c7b8bce`
**Phases**: single

### Open questions
- [ ] Vessel speed-over-ground passed as NaN (no offline SOG source wired in) — follow-up issue or accept the floored speed-dependent term?
- [ ] level_frame/tide_frame default to bizzy/base_link_north_up and bizzy/map_tide by convention; unverified against a real bag — acceptable given diagnostics + note will surface 100% missing_attitude/heave if wrong?

## Plan Review
**Status**: complete
**When**: 2026-09-11 14:47 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-55/plan.md` at `c7b8bce`
**PR**: PR-less (`--issue` mode; branch `feature/issue-55`, not pushed)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) `sidescan_viewer_window.cpp:112,934` includes the header item 3 deletes and shows `sounding_uncertainty_caveat()` in the CUBE-tuning dialog (#45 put it where the numbers are set) — build break plus a lost operator caveat; `CloudLoadOutcome::notes` is a different surface — `plan.md:85,149`
- [ ] (must-fix) `SidescanBagSession::readMbesWindow` has no TF lookup (it lifts with `MbesPing`'s pre-captured pose because `tf_buffer_` is a 30 s rolling cache filled only during the load scan); projecting from it at window time hits `lookupAtOrLatest`'s `TimePointZero` fallback and returns END-OF-BAG attitude with `missing_attitude == 0` — wrong data reported healthy — `plan.md:86-95`
- [ ] (must-fix) `MbesWindowOptions` carries no base-link frame (`mbes_window_reader.hpp:42-43`), so `ProjectorParams::base_link_frame` stays the library default `base_link` while level/tide are `bizzy/…` — the attitude lookup can never resolve — `plan.md:96-99`
- [ ] (must-fix) Missing attitude makes the errors NaN but leaves positions finite, so the range gate keeps the soundings, `report_projection_summary`'s frame-override warning (gated on `soundings == 0`) never fires, and the total loss shows only as `run_cube`'s "no beam geometry" note — decision 5's "degrades gracefully" is not honest for the estimator path — `plan.md:58-66,202-208`
- [ ] (must-fix) New fields named `*_error` entrench the misnomer rolker/cube_bathymetry#158 is open to rename (it already counts sites "across cube and the explorer"); these are new fields with no compatibility burden — name them `*_variance` or say in the plan that it defers to #158 — `plan.md:36-46,82-84`
- [ ] (must-fix) `ProjectionRunTotals`' `pings`/`beams`/`soundings` are not in `ProjectionDiagnostics`, so a summed-diagnostics field cannot fill the summary — `totals.pings` prints 0 and the zero-sounding warning can never fire — `plan.md:97-109`
- [ ] (must-fix) The retired `project_detections` skips beams with non-positive travel time; `ErrorModel::compute` has no such guard and the default `minimum_range = 0.0` keeps a zero-range sounding at the sonar head — set a non-zero minimum range or filter twtt in `project_ping` — `plan.md:75-77`
- [ ] (must-fix) `color_vocabulary.hpp:65-80` and `test_color_vocabulary.cpp:73-88` state as a verified gap that a sounding carries no uncertainty — false after this change; update the text and name (even if deferring) whether the point Uncertainty channel now opens — `plan.md:143-158`
- [ ] (suggestion) Extend `test_tf_lift.cpp`'s NaN-carry-through test to the two new fields instead of asserting the property in prose only — `plan.md:180`
- [ ] (suggestion) Decision 1's consumer list omits `sidescan_viewer_window.hpp` (4 sites incl. the self-cal `cube_soundings_`) and `cube_lab.{hpp,cpp}` — `plan.md:31-35`
- [ ] (suggestion) Match `Parameters::influenceRadius`'s own gate (`vertical_error <= 0` rejected, not just negative) — `cube_lab.cpp` casts a NaN radius to int for its loop bounds — `plan.md:110-117`
- [ ] (suggestion) Say that `horizontal_error` is radial/drms and feeds `influenceRadius`, so sounding spread — and the surface's look — changes with the real model — `plan.md:110-117`
- [ ] (suggestion) Add a `.agents/README.md` row for the new `mbes_projection.{hpp,cpp}`, not only the two edited rows — `plan.md:139-141,158`
- [ ] (verified, no change) NaN vessel speed is handled exactly as decision 4 claims: `error_model.cpp:205-218` floors the speed-dependent horizontal terms to 0; the note that it makes horizontal error optimistic is worth one line — `plan.md:52-57`


## Plan Authored
**Status**: complete
**When**: 2026-09-11 14:58 -04:00
**By**: Claude Code Agent (Claude Sonnet 5)

**Plan**: `.agent/work-plans/issue-55/plan.md` at `63164af` (revision 2)
**Branch**: feature/issue-55 at `63164af`
**Phases**: single

### Revision 2 — must-fixes resolved
- [x] `sidescan_viewer_window.cpp:112,934` build break / lost caveat — file added to Files-to-Change; dialog now includes `mbes_projection.hpp` and renders the new shared `offline_projection_caveat()` in place of `sounding_uncertainty_caveat()`
- [x] Plan item 4 (cloud path) removed from scope entirely per operator decision; `SidescanBagSession::readMbesWindow`'s TF-lookup-at-window-time hazard is now moot for this issue and explicitly deferred to #56
- [x] `MbesWindowOptions` gains `base_link_frame` (plus `level_frame`/`tide_frame`), with verified defaults `bizzy/base_link` / `bizzy/base_link_north_up` / `bizzy/map_tide` sourced from a real BizzyBoat M3 bag's TF tree (path recorded in the plan)
- [x] Honest degradation: `MbesWindowResult` now carries a full `cube::ProjectionRunTotals` (not just `ProjectionDiagnostics`) so `report_projection_summary`'s missing-attitude/heave counts always print in the load note; `cube_lab.cpp`'s "no beam geometry" skip note reworded to stop misattributing frame-mismatch drops to beam geometry, and points at the load note; one-line doc note added that NaN vessel speed floors the horizontal speed-dependent terms to 0, making horizontal error optimistic
- [x] New `MbesSounding` fields renamed `vertical_variance`/`horizontal_variance` (not `*_error`); plan states this leads cube_bathymetry#158 rather than following it; the `cube::Sounding::vertical_error`/`horizontal_error` -> renamed-field mapping happens once, inside the new `project_ping()` helper
- [x] `ProjectionRunTotals` totals-only fields — resolved by the same `MbesWindowResult` change above; `mbes_pass_loader.cpp` accumulates a running `ProjectionRunTotals` across passes instead of summing bare `ProjectionDiagnostics`
- [x] Non-positive travel time / zero-range sounding at the sonar head — `offline_projector_params()` sets `minimum_range = 0.05 m` (rationale and its limits stated; left as an Open Question whether 5 cm is right for the M3 specifically)
- [x] `color_vocabulary.hpp`/`test_color_vocabulary.cpp`'s now-false verified-gap text — reworded; states real variances exist on some cloud sources after this issue but the Uncertainty colour channel itself opens in #56, once both cloud sources carry them

### Suggestions folded in
- [x] `test_tf_lift.cpp` extended to cover the two new fields
- [x] Decision 1's consumer list corrected/completed: `sidescan_viewer_window.hpp` (4 sites incl. `cube_soundings_`) and `cube_lab.{hpp,cpp}` added
- [x] `cube_lab.cpp`'s drop gate now reuses `Parameters::influenceRadius`'s own gate (rejects `vertical_variance <= 0`, not just negative) by branching on `influenceRadius`'s NaN result directly, instead of a separate hand-rolled check
- [x] Plan states `horizontal_variance` is radial/drms and feeds `influenceRadius`, so sounding spread and the surface's look change under the real model
- [x] `.agents/README.md` gets a new row for `mbes_projection.{hpp,cpp}`

### Correction found during revision (not a review finding, self-caught)
Revision 1 had the deletion scope backwards for the test suite: it said
delete `test/test_mbes_geometry.cpp`, but decision 1 (keep
`project_beam`/`project_detections` alive for the cloud path until #56)
means that file's subject survives this issue — revision 2 keeps it
unchanged. Also documented, in Context, that `mbes_pass_loader`'s output
feeds the region-selection 3D point cloud directly (not only
`cube_lab::run_cube`), so the operator's "cloud view keeps its placeholder"
framing needed the qualification that ships in revision 2's Context section
(data now flows through for that cloud source; only the colour-channel UI
wiring is deferred).

### Open questions
- [ ] Vessel speed-over-ground passed as NaN (no offline SOG source wired
  in) — follow-up issue or accept the floored, optimistic speed-dependent
  horizontal term?
- [ ] `minimum_range = 0.05 m` is a plan-time choice, not verified against
  the M3's near-field spec — revisit if a real bag shows legitimate
  detections inside 5 cm.

## Plan Review
**Status**: complete
**When**: 2026-09-14 10:22 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-55/plan.md` at `63164af` (revision 2)
**PR**: PR-less (`--issue` mode; branch `feature/issue-55`, not pushed)
**Verdict**: changes-requested

Revision 2 resolves all 8 prior must-fixes and folds in every suggestion;
each claim it makes about the code was re-verified against source and holds
(`sidescan_viewer_window.cpp:112,934`; `mbes_pass_loader.cpp:65` passing
default `MbesWindowOptions`; `ProjectionRunTotals`' counts vs
`ProjectionDiagnostics`; `color_vocabulary.hpp:65-80`; the CMake
registrations; the `.agents/README.md` rows). Two structural risks it does
not yet carry, plus a test gap, are below.

### Findings
- [ ] (must-fix) No test is planned for the new `mbes_projection.{hpp,cpp}` — the `vertical_error`→`vertical_variance` mapping, `offline_projector_params()`'s `minimum_range`/frame defaults, and the diagnostics passthrough are new production code; CMake gains the `.cpp` but registers no `test_mbes_projection`, and `test_mbes_window_reader.cpp`/`test_mbes_pass_loader.cpp` (whose types and behaviour change) are absent from Files-to-Change and today hold only pure-math/error-path tests — `plan.md:259-287,333`
- [ ] (must-fix) `minimum_range = 0.05` does not replace the guard it retires: `project_detections` skipped `twtt <= 0` **or** `sound_speed <= 0`, but cube's gate is `range_sq in [min^2, max^2]`, so a NEGATIVE travel time or sound speed yields a mirrored sounding whose squared range passes the gate and reaches the estimator — only the exactly-zero case is caught, so the rationale's "excluding exactly the non-positive-range sentinel case" is inaccurate; filter non-positive twtt/sound speed in `project_ping` as well — `plan.md:137-147`
- [ ] (must-fix) The real model changes CUBE's influence radius by ~10x and the plan does not say so: `cube::Vessel{}`'s `gps_drms = 2.0` enters `swath_horizontal` as a constant `total_gps_variance = 4.0 m^2` on every sounding (`error_model.cpp:105,395`), so `horizontal_error >= 4 m^2` always and `Parameters::influenceRadius` caps spread at `CONF_99PC*sqrt(h) ~ 5.2 m` against the placeholder's `0.2 + 0.01*depth` -> ~0.5 m; `run_cube`'s spread loop is O(radius^2/cell^2) per sounding, so up to ~100x more node inserts on a run already measured in minutes over ~1M soundings. State the magnitude, decide whether to override `gps_drms` for an RTK boat rather than take the generic-GPS default, and name the 2 m GPS assumption in `offline_projection_caveat()` — it outweighs "lever arms zero" — `plan.md:84-89,154-160,250-258`
- [ ] (must-fix) Approach item 7 captures `report_projection_summary`'s two streams but appends only the summary line, discarding the `err` stream — which is where the default-beamwidth warning lives, and `kongsberg_em_bridge` leaves `rx_beamwidths` empty on every M3 ping, so that warning fires on 100% of beams in exactly this deployment; append the warnings too — `plan.md:206-217`
- [ ] (suggestion) `read_mbes_window`'s existing `skipped_pings` (ping dropped for a missing world<-sensor TF, before projection) has no home in `ProjectionRunTotals` and so never reaches the note — surface it alongside the summary so both drop populations are visible — `plan.md:191-205`
- [ ] (suggestion) Accumulate `beams` from `ProjectionDiagnostics::total` (`ErrorModel::compute` emits exactly one sounding per beam and never skips) rather than a separately-counted "ping's beam count", so the denominator cannot drift from the numerators — `plan.md:197-199`
- [ ] (suggestion) Make `minimum_range`'s 0.05 a named constant in `mbes_projection.hpp` carrying the rationale, so the open question has one site to revisit — `plan.md:137-147`
- [ ] (verified, no change) `cube::Sounding::sonar_relative_position` is pure sonar-frame geometry identical to `project_beam`'s formula, so lifting the projector's output with `tf_lift` does not double-apply attitude; `read_mbes_window` fills its TF buffer for the whole window before reading any detections, so the prior review's "END-OF-BAG attitude reported healthy" hazard does not recur on this path (a frame mismatch raises LookupException, counted as `missing_attitude`); and `run_cube` is fed only from `load_cloud_passes`, so no NaN-variance soundings arrive from the untouched cloud path

## Plan Authored
**Status**: complete (revision 3)
**When**: 2026-09-14 11:05 -04:00
**By**: Claude Code Agent (Claude Fable 5.1)

**Plan**: `.agent/work-plans/issue-55/plan.md` at `0722993`
**Source**: Plan Review round 2 at `6ce1992` (changes-requested: 4 must-fix, 3 suggestions)

### Resolved
- [x] (must-fix) tests: new `test/test_mbes_projection.cpp` + additions to `test_mbes_window_reader.cpp` / `test_mbes_pass_loader.cpp` — Approach item 13, Files to Change
- [x] (must-fix) validity guard: `project_ping` drops `sound_speed <= 0` pings and `twtt <= 0` beams (zero and negative) with `invalid_pings`/`invalid_beams` counts — Approach item 1
- [x] (must-fix) GPS default / influence radius: decision 7 — operator (2026-09-14) chose library defaults, matching the boat and every offline tool; 2 m GPS named first in the caveat; runtime measured on a real pass (Approach item 12); general fix parked as unh_marine_autonomy#385
- [x] (must-fix) `err` stream of `report_projection_summary` appended to the load note — Approach item 7
- [x] (suggestion) `skipped_pings` + invalid counts surfaced in the note — Approach item 7
- [x] (suggestion) `beams` accumulated from `ProjectionDiagnostics::total` — Approach item 6
- [x] (suggestion) `kOfflineMinimumRangeM` named constant — Approach item 1, Open Questions

### Findings
- [ ] Ready for Plan Review round 3

## Plan Review
**Status**: complete
**When**: 2026-09-14 10:59 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-55/plan.md` at `0722993` (revision 3)
**PR**: PR-less (`--issue` mode; branch `feature/issue-55`, not pushed)
**Verdict**: changes-requested

All four round-2 must-fixes and all three suggestions are present in revision 3,
and each new claim was re-verified against source: decision 7 is accurate
(`detections_to_pointcloud.cpp:128-137` sets exactly `ellipsoidal_referenced`,
`range_error_percent`, `range_error_floor_m` on `Vessel`/`Device`;
`bizzyboat.yaml:100-104` sets only the three frame names; `import_bag_main.cpp:819`,
`batch_regen_main.cpp:531`, `bag_to_geotiff.cpp:286` all default-construct
`ProjectorParams`; `error_model.h:109` `gps_drms = 2.0`; `error_model.cpp:105,395`;
`parameters.cpp:80-102`'s gate and `CONF_99PC*sqrt(h)` cap all read as the plan
states). `ErrorModel::compute` (`error_model.cpp:440-480`) is indeed 1:1 per beam,
so the `beams += ProjectionDiagnostics::total` suggestion holds. Two structural
findings below; the first is a correctness error in the new guard.

### Findings
- [ ] (must-fix) The per-beam drop is index-based, but `DetectionsProjector::project` applies its range gate BEFORE returning (`detections_projector.cpp:200-209`), so `result.soundings[i]` is not beam `i` whenever any beam is range-filtered — and with `minimum_range = 0.05` plus NaN-position beams (absent/non-finite `rx_angles`) that is the normal case, so the wrong sounding gets dropped; drop by VALUE instead — `cube::Sounding::slant_range` is `twtt * sound_speed / 2` (`sounding.h:45-49`), so `slant_range <= 0 || !isfinite` is an exact, index-free witness of the retired `twtt <= 0` guard — `plan.md:186-190`
- [ ] (must-fix) Target-level build consequence missing: `mbes_window_reader.cpp` lives in `sidescan_core`, whose `SIDESCAN_CORE_DEPS` (`CMakeLists.txt:105-116`) does NOT include `cube_bathymetry`, and every cube_bathymetry consumer in this file also needs the `CUBE_BATHYMETRY_INCLUDE_ROOT` SYSTEM include (`CMakeLists.txt:26-29`) because the exported interface misses a level. Putting `mbes_projection.cpp` in the library and `cube::ProjectionRunTotals` in the public `MbesWindowResult` propagates that to `sidescan_probe`, `test_mbes_window_reader`, `test_mbes_pass_loader` and `test_session_index_io`. The Context line "already a `find_package`d dependency — no new dependency" is true per-package and false per-target; say which targets gain `cube_bathymetry` + the include root — `plan.md:28-29,426`
- [ ] (suggestion) State the note's arithmetic: an invalid beam dropped after projection PASSED the range gate, so it is counted in `beams` and in neither `soundings` nor `filtered_range`; say `beams = soundings + filtered_range + invalid_beams` so the load note reconciles instead of appearing to lose beams — `plan.md:186-190,242-254`
- [ ] (suggestion) Approach item 12's measurement has no headless path: `run_cube`'s only non-GUI caller is `test_cube_lab.cpp`; the survey-explorer GUI is the only thing that loads a real multi-pass selection. Name how the before/after timing is taken (a temporary bench target over `load_cloud_passes` + `run_cube`, or an operator-run GUI measurement) or the number will be quietly skipped at implementation time — `plan.md:375-383`
- [ ] (suggestion) `kOfflineMinimumRangeM = 0.05` is itself a divergence from the production store decision 7 otherwise matches: the live node's `minimum_range` default is `0.0` and BizzyBoat does not override it (`detections_to_pointcloud.cpp:347`, `bizzyboat.yaml:100-104`; lr30 sets 5.0). Negligible in effect, but one clause in the caveat/plan keeps "the lab runs what the boat runs" honest — `plan.md:171-178`
- [ ] (suggestion) `test_tf_lift.cpp:52` `IdentityTransformCarriesEveryField` is the test whose name promises every field; extend it to the two new fields as well, not only the NaN and rotated-lift cases — `plan.md:339-344`
- [ ] (verified, no change) Round-2's four must-fixes and three suggestions are all genuinely present in the plan text, not merely claimed in the Plan Authored entry; every referenced issue exists as described (marine_perception_tools#56, unh_marine_autonomy#385, cube_bathymetry#158/#145), the 2026-08-20 bag path in decision 6 resolves on the NAS, and the cited line numbers in `sidescan_viewer_window.cpp:112,934`, `cube_lab.cpp:305-330`, `test_cube_lab.cpp:545-552`, `test_tf_lift.cpp` and `CMakeLists.txt` all match
