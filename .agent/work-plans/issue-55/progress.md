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
