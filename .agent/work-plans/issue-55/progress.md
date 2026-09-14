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

## Plan Authored
**Status**: complete (revision 4)
**When**: 2026-09-14 11:40 -04:00
**By**: Claude Code Agent (Claude Fable 5.1)

**Plan**: `.agent/work-plans/issue-55/plan.md` (revision 4, commit after `b76602d`)
**Source**: Plan Review round 3 at `b76602d` (changes-requested: 2 must-fix, 4 suggestions; both must-fixes mechanical, so folded in by the host and carried to the implementation checkpoint rather than a fourth review round)

### Resolved
- [x] (must-fix) invalid beams dropped by value (`slant_range <= 0 || !isfinite`), not by index — Approach item 1
- [x] (must-fix) `cube_bathymetry` dep + SYSTEM include added to `sidescan_core` as PUBLIC so `sidescan_probe`, `test_mbes_window_reader`, `test_mbes_pass_loader`, `test_session_index_io`, `test_cube_lab` inherit it — Files to Change (CMakeLists.txt)
- [x] (suggestion) load-note arithmetic `beams = soundings + filtered_range + invalid_beams` stated — Approach item 1
- [x] (suggestion) runtime measurement driven through the explorer, wall-time line in the notes — Approach item 12
- [x] (suggestion) `kOfflineMinimumRangeM` noted as a small divergence from production's `0.0` — Approach item 1
- [x] (suggestion) `IdentityTransformCarriesEveryField` extended to the two new fields — Files to Change

### Findings
- [ ] Awaiting the operator's implementation checkpoint

## Implementation

**Status**: complete
**When**: 2026-09-14 11:37 -04:00
**By**: Claude Code Agent (Claude Opus)
**Branch**: feature/issue-55 at c72a47f

### Findings

- [x] **Approach 1 — shared offline helper.** New `src/mbes_projection.{hpp,cpp}`: `kOfflineMinimumRangeM = 0.05` (named, with the divergence-from-production note), `offline_projector_params()` (caller frames + library-default vessel/device), `project_ping()` (the one `cube::Sounding::{vertical,horizontal}_error` -> `MbesSounding::{vertical,horizontal}_variance` rename site, plus the ping/beam validity guard), and `offline_projection_caveat()` — 2 m GPS first, then lever arms / generic device, then the NaN-speed optimism, pointing at unh_marine_autonomy#385.
- [x] **Approach 2 — `MbesSounding` extended** with NaN-defaulted `vertical_variance` / `horizontal_variance`; `mbes_geometry.hpp`'s header comment rewritten for the ownership split; `project_beam`/`project_detections` kept for the cloud path until #56.
- [x] **Approach 3 + 4 — placeholder retired.** `src/sounding_uncertainty.hpp` and `test/test_sounding_uncertainty.cpp` deleted; `sidescan_viewer_window.cpp:112` now includes `mbes_projection.hpp` and `:934` shows `offline_projection_caveat()`.
- [x] **Approach 5 + 6 — reader rewired.** `MbesWindowOptions` gains the three verified `bizzy/`-namespaced projector frames; `MbesWindowResult` gains `cube::ProjectionRunTotals diagnostics` (with `reports_georeferencing = false`), `invalid_pings`, `invalid_beams`; `read_mbes_window` holds one `DetectionsProjector` and calls `project_ping` with NaN speed-over-ground.
- [x] **Approach 7 — load note.** `mbes_pass_loader` sums each pass's totals and appends cube's own `report_projection_summary()` output — the summary line AND every line of the `err` stream — plus the skipped/invalid line and the caveat, once per load.
- [x] **Approach 8 — `run_cube` rewired.** Reads the variances straight into `cs.vertical_error`/`horizontal_error`; the drop gate is now `params.influenceRadius(cs)`'s own NaN result (which also rejects a ZERO vertical variance, the case the retired hand-rolled check let through); skip note reworded to "invalid uncertainty (see load notes ...)"; both doc comments rewritten.
- [x] **Approach 9 — test disposition** as planned: `test_mbes_geometry.cpp` untouched, `test_sounding_uncertainty.cpp` deleted with its subject, `test_cube_lab.cpp` stamps variances via a local test-only copy of the retired formula (so the characterisation numbers stay comparable), `test_tf_lift.cpp` extended on all three carry-through tests, CMake registration swapped.
- [x] **Approach 10 — `color_vocabulary.hpp`** reworded: the channel stays greyed for points, but because only the CUBE-lab source carries variances (the scrub window does not) and both must move together at #56 — not because "a sounding carries no uncertainty", which is now false. `test_color_vocabulary.cpp` updated, plus a new test asserting the reason names #56 and no longer says "placeholder".
- [x] **Approach 11 — `.agents/README.md`**: `sounding_uncertainty.hpp` row removed, `mbes_geometry.hpp` re-described, `mbes_projection.{hpp,cpp}` row added.
- [x] **Approach 13 — tests.** New `test/test_mbes_projection.cpp` (10 tests). `test_mbes_window_reader.cpp` +3 (frame defaults; every-field accumulation; the georeferencing claim). `test_mbes_pass_loader.cpp` +4 (all four note parts, caveat exactly once; no georeferencing clause; silence when nothing projected; silence through the loader).
- [x] **Build**: clean. Zero compiler warnings on the changed files under `-Wall -Wextra -Wpedantic`. Linters all green: uncrustify 97/97, cpplint 97/97, copyright 100/100, lint_cmake 1/1, xmllint 1/1 — nothing disabled or suppressed.
- [x] **Tests**: `796 tests, 0 errors, 0 failures, 97 skipped`, run twice back to back. Per new/changed binary: `test_mbes_projection` 10/10, `test_mbes_window_reader` 14/14, `test_mbes_pass_loader` 11/11, `test_cube_lab` 23/23, `test_color_vocabulary` 7/7, `test_tf_lift` 4/4, `test_survey_explorer_window` 41/41.
- [x] **Note on the test environment**: 20 `ExplorerWindowFixture` failures seen on the first runs were environmental, not code — a stale sandbox scaffold from an earlier session left a self-referential symlink at `/tmp/tmp.ldBwWOxijY/isolatedbin/printf`, and `::testing::TempDir()` defaults to `/tmp`, so the explorer's basemap-layer discovery walked into an ELOOP. They all pass with `TMPDIR` set to a clean directory. Worth knowing separately: `SidescanViewerWindow::discoverBasemapLayers` calls `e.is_directory()` without the `error_code` overload inside a `directory_iterator(dir, ec)` loop, so one bad symlink anywhere under the stores root throws out of the scan — a real robustness gap, out of scope here, not filed.
- [ ] **DEVIATION 1 (Approach 1 / 13) — a ZERO travel time lands in `filtered_range`, not `invalid_beams`.** Its range is 0, below `kOfflineMinimumRangeM`, so `DetectionsProjector::project` filters it before `project_ping` sees it; only a NEGATIVE travel time reaches the signed slant-range test. Both are dropped, both counted, and `beams = soundings + filtered_range + invalid_beams` closes either way. The plan predicted both would land in `invalid_beams`; the tests pin each to the counter that actually receives it, and the plan is edited inline to say so.
- [ ] **DEVIATION 2 (Approach 6 / 7) — two named helpers instead of inline loops.** `accumulate_ping()` (`mbes_window_reader.hpp`) and `append_projection_notes()` (`mbes_pass_loader.{hpp,cpp}`) were extracted so Approach item 13's reader and loader tests could run without a bag fixture — the split item 13 explicitly allowed for. Recorded inline in the plan.
- [ ] **DEVIATION 3 — `test/test_survey_explorer_window.cpp` was not in the plan's Files to Change, and needed two changes.** (a) All three synthetic bag writers hang `bizzy/base_link` straight off `bizzy/map`, so they carry no attitude chain (`base_link_north_up <- base_link`) and no heave chain (`map_tide <- base_link`). `ACubeRunAddsASurfaceOverTheSelectionCloud` failed with "the CUBE run never produced a surface" — the honest-degradation path of Approach item 7 behaving exactly as designed, caught by the first end-to-end test to reach it. All three writers now build the real tree shape, not only the one whose test failed. (b) `ClosingDuringACloudLoadReturnsPromptly`'s baseline load measured ~4.7 s against `process_until`'s 5 s ceiling — green alone, red under a parallel colcon run — because the load now runs the full error model over ~2M beams. The fixture's ping count is halved (baseline ~2.7 s); the test's own `baseline_ms > 300` guard still fails loudly if it stops being a long job. Raising the ceiling would have hidden the cost instead of making room for it. Both recorded inline in the plan.
- [x] **Approach 12 (the instrument) — wall time is in.** `run_cube`'s note now reads "N of M nodes estimated in T.TT s" from a `steady_clock` span around the whole run, pinned by `RunCube.ReportsItsOwnWallTime`.
- [ ] **OWED TO THE OPERATOR — Approach 12's actual measurement on a real pass.** It needs the GUI driven by hand over an archived bag, which this agent could not do headlessly; a fabricated number would be worse than an empty row. **Steps**: (1) `survey_explorer --index ~/data/world/survey_index.db`; (2) navigate to the 2026-08-20 BizzyBoat `bizzy_timing` recording (the bag decision 6's frames were verified against) or a `bizzyboat_sonar` recording; (3) left-drag a region covering several overlapping passes; (4) wait for the multi-pass load and READ ITS NOTE — the projection summary, the drop counts and the caveat are all there now; (5) set the cell size to the lab default and click Run CUBE; (6) read "... nodes estimated in T.TT s" off the run note, together with the sounding count from the legend. Record wall time + sounding count in the PR body. **File a performance follow-up only if that number makes the lab unusable, with the number in it** — not on the ~100x prediction alone (plan decision 7).
- [x] **One measured datapoint exists meanwhile**, from the fixture above rather than from a real pass: a windowed load of ~2.05M synthetic soundings (4000 pings x 512 beams) takes ~4.6-4.9 s wall through the real projector on this machine, offscreen. That is the PROJECTION cost, not `run_cube`'s spread cost — which is the half decision 7 is actually worried about, and the half still owed.
- [x] **Not re-opened**: operator decisions 1-7 stand as settled. No SOG source was added (decision 5 / Open Question 1); `kOfflineMinimumRangeM` stays 0.05 m (Open Question 2); library-default vessel/device is untouched (decision 7).
- [x] **11 atomic commits**, one logical change each, all under the agent identity, no `--no-verify`. Nothing pushed.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-09-14 12:55 -04:00
**By**: Claude Code Agent (Claude Fable 5.1)
**Verdict**: changes-requested

**Branch**: feature/issue-55 at `8217d1d`
**Mode**: pre-push
**Depth**: Deep (reason: large change, new cross-package dependency at target level)
**Must-fix**: 5 | **Suggestions**: 11
**Round**: 1 | **Ship**: continue — five must-fixes, all mechanical; one fix round expected

### Findings
- [ ] (must-fix) caveat/doc claim "spreads each sounding over ~5 m" is false: `influenceRadius` subtracts the horizontal term and caps at 5.15 m; at lab cell sizes the radius equals the cell; the real model spreads LESS than the placeholder and the ~100× slowdown does not exist — reword `offline_projection_caveat()`, `cube_lab.hpp` doc, `cube_lab.cpp` comment, test comment, plan decision 7 — `src/mbes_projection.cpp:95`, `src/cube_lab.hpp:186`, `src/cube_lab.cpp:197`
- [ ] (must-fix) root README §"Per-sounding uncertainty in the lab" still documents the deleted placeholder — `README.md:255-285`
- [ ] (must-fix) IHO-order combo tooltip still says "PLACEHOLDER per-sounding error" — `src/sidescan_viewer_window.cpp:734`
- [ ] (must-fix) load note (~1.5 kB) joined into a clipping `QLabel` with no tooltip; add `setToolTip` with the full notes — `src/sidescan_viewer_window.cpp:1204,2008`
- [ ] (must-fix) no explicit line when `missing_attitude > 0`; positions stay finite so the library frame warning never fires and the CUBE run silently drops all — `src/mbes_pass_loader.cpp:56`
- [ ] (suggestion) two elapsed times in one status sentence — `src/cube_lab.cpp:399`, `src/sidescan_viewer_window.cpp:1204`
- [ ] (suggestion) CLI wording ("--*-frame overrides", README) forwarded into the GUI note — `src/mbes_pass_loader.cpp:65`
- [ ] (suggestion) invalid-sound-speed pings counted in `pings`, misattributing an all-bad bag to frames — `src/mbes_window_reader.cpp:237`
- [ ] (suggestion) totals accumulate before clip/geo-skip; comment says "what the operator actually got" — say "projected" — `src/mbes_pass_loader.cpp:104,128`
- [ ] (suggestion) early return on `pings == 0` hides the skipped-pings line — `src/mbes_pass_loader.cpp:40`
- [ ] (suggestion) "the one place the two spellings meet" is false; `cube_lab.cpp` translates back — `src/mbes_geometry.hpp:75`, `src/mbes_projection.cpp:80`
- [ ] (suggestion) "kongsberg_em_bridge reports no beamwidths" → "currently" (marine_tools#85) — `src/mbes_projection.cpp:596`, `.agents/README.md:53`
- [ ] (suggestion) plan sync: item 13 zero-twtt text; `abandon()` zeroing diagnostics — `.agent/work-plans/issue-55/plan.md`
- [ ] (suggestion) PR body: issue scope item 2 half met by design; decide whether this PR closes mpt#51 (its 0.05 m floor code is deleted)
- [ ] (suggestion) plan ADR table repo labels; `.agents/README.md` test tree row for `test_mbes_projection` — `.agents/README.md:58`
- [ ] (suggestion) default-beamwidth stated twice per note (library WARNING + caveat item 2) — `src/mbes_projection.cpp:97`
- [ ] (candidate, proposal only) `.agents/README.md` Common Pitfalls: `cube::ProjectorParams` defaults to unprefixed `base_link`, which BizzyBoat's alias resolves silently to NaN variances; synthetic test bags need the `base_link_north_up ← base_link` and `map_tide ← base_link` chains

## Implementation

**Status**: complete
**When**: 2026-09-14 12:09 -04:00
**By**: Claude Code Agent (Claude Opus)
**Source**: Local Review (Pre-Push) round 1 at 8217d1d
**Branch**: feature/issue-55 at b382aa7

All 5 must-fixes and 9 of the 11 suggestions are addressed in 7 atomic
commits. Build warning-clean; 804 tests, 0 failures, 97 skipped (796 before,
+8 new); `ament_cpplint` and `ament_uncrustify` clean on every changed file.

**One environment note for whoever runs the suite next**: 22 window tests fail
with `filesystem error: status: Too many levels of symbolic links
[$TMPDIR/isolatedbin/printf]` when `TMPDIR` points at an agent sandbox's
scratch directory — the bag fixtures walk `temp_directory_path()`. Running
with a plain `TMPDIR` is green. Nothing in the package changed to cause it.

### Resolved

- [x] (must-fix) The false "~5 m influence radius" / "ten times further" /
  "~100× slowdown" claim — `ff51b28`. `Parameters::influenceRadius`
  (cube_bathymetry `parameters.cpp:72-110`) computes
  `distance_scale·(ratio-1)^(1/distance_exponent) − CONF_99PC·√horizontal_error`,
  caps at that same horizontal term and floors at `distance_scale` (the cell
  size) LAST, so the floor wins. The horizontal term is SUBTRACTED: the 4 m²
  floor `cube::Vessel{}`'s `gps_drms = 2.0` puts under every sounding makes it
  spread LESS, not more; ~5.15 m is a ceiling, reached only at coarse cells
  under a loose vertical budget. Corrected in `offline_projection_caveat()`,
  `cube_lab.hpp`, `cube_lab.cpp`, `test_cube_lab.cpp`, and plan decision 7 +
  Approach item 12. The 2 m GPS assumption itself (operator decision 7) still
  leads the caveat — only its consequence changed. `OfflineProjectionCaveat`
  gains a test pinning the corrected wording and refusing the old claim's
  return.
- [x] (must-fix) README §"Per-sounding uncertainty in the lab" documented the
  deleted placeholder — `d98cb9f`. Rewritten for the real chain
  (DetectionsProjector + ErrorModel) and the three library-default
  assumptions in the caveat's own order. The colour-channel paragraph's
  reason for greying **Uncertainty** is brought level with
  `color_vocabulary.hpp`'s (not "a sounding carries none" but "not every
  cloud carries one yet", mpt#56).
- [x] (must-fix) IHO-order tooltip said "PLACEHOLDER per-sounding error" —
  `d98cb9f`, with a window test pinning the wording out.
- [x] (must-fix) The ~1.5 kB load note clipped in a width-Ignored `QLabel` —
  `d1e51e3`. All 46 status writes (and the opening message) now go through
  `SidescanViewerWindow::setStatusText()`, which mirrors the whole line into
  the label's tooltip. One helper rather than one tooltip at the note's site:
  a tooltip set once outlives the text it described and would report a status
  two operations old. Window test asserts tooltip == text and that it follows
  a later, shorter status.
- [x] (must-fix) No explicit line when `missing_attitude > 0` — `547395d`.
  The note now names the count, the `level <- base_link` chain from
  `MbesWindowOptions`, and the consequence (soundings load, draw and colour
  normally; a CUBE run drops every one). Pinned in `test_mbes_pass_loader`,
  with a companion test that it says nothing on a clean load.
- [x] (suggestion) Two elapsed times in one status sentence — `6f52490`.
  "… in 0.42 s in 31.2 s" now distinguishes the estimator's own wall time
  (run_cube's note) from the whole job's, bag reads included.
- [x] (suggestion) CLI wording forwarded into the GUI — `547395d`. cube's
  `pings > 0 && soundings == 0` warning tells the reader to check
  "`--*-frame` overrides" against a README section; this window has no such
  flags. Only that sentence is restated, naming the three compiled-in frames;
  cube's counts in front of it stay verbatim so they cannot drift from the
  offline tools'. Test asserts the note carries neither `--*-frame` nor
  `README`, and does carry the frame names.
- [x] (suggestion) Invalid-sound-speed pings counted in `pings` — `9fb89eb`.
  `ProjectionRunTotals::pings` means "pings handed to
  `DetectionsProjector::project()`"; a refused ping never was. It is counted
  once, in `invalid_pings`. This was the finding's real sting: counting it in
  both made cube's zero-soundings warning blame the FRAMES for a bag of bad
  sound speeds. Two tests, including the all-bad-bag case end to end.
- [x] (suggestion) Totals comment claimed "what the operator actually got" —
  `9fb89eb`. They are accumulated before the contact clip and before a
  geo-anchorless pass can be skipped, so they describe the PROJECTION;
  `sounding_counts` describes what survived into the cloud.
- [x] (suggestion) Early return on `pings == 0` hid the drop line —
  `547395d`. The note is silent only when nothing was projected AND nothing
  was dropped. It hid exactly the case it mattered for: an all-bad-sound-speed
  bag, whose pings are not in `totals.pings` at all.
- [x] (suggestion) "The one place the two spellings meet" was false —
  `6f52490`. There are exactly two translation sites, one each way:
  `project_ping()` inbound from cube, `run_cube()` outbound back to it. Each
  now names the other, which is what makes the claim checkable when
  cube_bathymetry#158 lands.
- [x] (suggestion) "kongsberg_em_bridge reports no beamwidths" → "currently",
  with marine_tools#85 — `6f52490`, in the caveat, the note comment, the test
  comment, the README and the `.agents/README.md` row.
- [x] (suggestion) Plan sync: item 13's zero-twtt bullet (it lands in
  `filtered_range`, not `invalid_beams`, as item 1's as-built note already
  said), `abandon()`'s diagnostics reset, and the round-1 changes recorded
  against items 6, 7 and 8 — `b382aa7`.
- [x] (suggestion) Plan ADR table now labels the repo (cube_bathymetry
  ADR-0007, workspace ADR-0008 — different documents with adjacent numbers);
  `.agents/README.md` gains a `test_mbes_projection.cpp` test-tree row —
  `b382aa7` / `6f52490`.
- [x] (suggestion, host's item) mpt#51: the plan's Open Questions now records
  that deleting `sounding_uncertainty.hpp` DISSOLVES mpt#51's 0.05 m-floor
  question rather than answering it — the floor is gone and
  `cube::ErrorModel` has no equivalent, so there is no decision left to make.

### Deferred

- [ ] (suggestion) "Default beamwidth stated twice per note" — **KEPT
  DELIBERATELY, and this is the reason**: the library WARNING line does NOT
  always carry the fact. `report_projection_summary` emits it only when
  `default_beamwidth_beams > 0`. On a bag whose driver DOES publish per-beam
  beamwidths (what marine_tools#85 is for), the warning is absent, and the
  caveat's item 2 would then be the only place the operator learns that the
  lab falls back to a generic device beamwidth when the ping carries none.
  Dropping it would make the note's completeness depend on the data. The
  duplication is one clause on today's M3 bags and load-bearing on tomorrow's.
- [ ] (candidate) `.agents/README.md` Common Pitfalls entry on
  `cube::ProjectorParams`' unprefixed `base_link` default — not added, per
  the task's instruction (it was raised as a proposal only).
- [ ] (host) The PR body still owes: whether this PR closes mpt#51, and the
  runtime measurement Approach item 12 asks for. The measurement's RATIONALE
  changed with the must-fix-1 correction — it is now a confirmation of no
  regression, not the sizing of a ~100× one — but it is still owed, because
  an unmeasured expectation of no regression is still an expectation.
