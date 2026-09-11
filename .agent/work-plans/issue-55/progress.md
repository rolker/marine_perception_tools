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
