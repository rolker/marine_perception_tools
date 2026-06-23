---
issue: 7
---

# Issue #7 — Offline georeferenced sidescan viewer + manual target tracker

## Plan Authored
**Status**: complete
**When**: 2026-06-21
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-7/plan.md` at `018f057`
**Branch**: feature/issue-7 at `018f057`
**Phases**: 3 stacked PRs (scaffold+ingest+projection / canvas+render+scrub / contacts+store+overlay), plus fast-follow PRs (echogram, store-swap to contact_manager #167, bathy-altitude)

### Open questions
- [ ] GGGS scope: v1 paints swath into an in-app map-frame raster (grid_map_core) + geodesy only at export boundary; OK to defer full GGGS tile-store sharing to fast-follow?
- [ ] Store format: serialize ContactArray as CDR (msg-fidelity, closest to #167) vs YAML (human-editable) — lean CDR.
- [ ] Echogram (fast-follow): reuse rqt_marine_sonar C++ Qt echogram widget by extraction vs reimplement.
- [ ] Confirm marine_acoustic_msgs (rosdep, not a source package here) is present on the operator-station build.

## Plan Review
**Status**: complete
**When**: 2026-06-21 23:54 -0400
**By**: Claude Code Agent (Claude Opus) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-7/plan.md` at `c48b8fb`
**PR**: PR-less (file-path / --issue mode)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) Frame names are generic, not pinned to the real bag. Plan §2 reads nav from `/bizzy/odom`+TF and §3 projects sensor pose "in `map`"; issue body says "ROS `map` frame". But the existing `BagSession` default `world_frame` is `bizzy/map_tide` (namespaced) — see `src/bag_loader.hpp:122` (`BagLoadOptions.world_frame = "bizzy/map_tide"`). v1 painting into an "in-app map-frame raster" must commit to one verified world/odom/sensor frame triple (verified against the real sidescan bag), not the bare `map`/`/bizzy/odom` shorthand, or projection lands in the wrong frame — `plan.md:30,32,36`
- [ ] (must-fix) Critical-path dep `marine_acoustic_msgs` (RawSonarImage) is left as an open question — confirmed NOT a source package in the workspace (rosdep only). PR1 (ingest) cannot compile without it. Resolve before PR1 starts, not "likely yes" — `plan.md:24,95`
- [ ] (suggestion) `down` sidescan channel under-specified. Issue lists port/starboard/**down** topics; plan only does port/starboard swath projection and `source="sidescan.port"`. State whether down-look is projected, used for nadir-altitude only, or descoped for v1 — `plan.md:30,38`
- [ ] (suggestion) Contact field completeness: `Contact.msg` documents `existence_probability = 1.0` for human-drawn contacts and `geo_pose` unresolved-when-`latitude==NaN`; plan's marking step (§6) omits both. Set `existence_probability=1.0` and resolve `geo_pose` (don't leave NaN) when boxing a contact — verified against `marine_interfaces/msg/Contact.msg` on main — `plan.md:38`
- [ ] (suggestion) Stationary-ping cap and "skip fully-covered ping" thresholds are named but unvalued. Expose as constants/params so they're tunable against the real Massabesic bag rather than hardcoded — `plan.md:33,52`
- [ ] Reuse/ADR alignment is otherwise solid: `BagSession`/`buffer_policy.hpp`/`cv_qt.hpp` names verified real; `tuner_core`+Qt-app split mirrors the actual CMake; `Contact`/`ContactArray` confirmed merged in `marine_interfaces` on main; ADR-0005/0006 (sidescan store) and ADR-0007 (GeoCoder) correctly scoped as non-blocking; GGGS-descope + CDR-store decisions resolved with Roland.

## Implementation
**Status**: complete (PR1 of 3)
**When**: 2026-06-22
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Commit**: `0522f58` on feature/issue-7 (local; not pushed)
**Scope**: PR1 = sidescan_core ingest + projection.
- `src/sidescan_geometry.hpp` — pure projection (slant→ground, sample→bizzy/map XY, nadir altitude); 9 gtests, all pass.
- `src/sidescan_bag_session.{hpp,cpp}` — rosbag2 ingest of RawSonarImage(port/stbd/down)+/tf; per-ping TF pose, along-track distance, nearest-nadir altitude, window-by-distance.
- `src/sidescan_probe.cpp` — headless end-to-end CLI.
- CMake: `sidescan_core` lib + `sidescan_probe` exe + `test_sidescan_geometry`; `marine_acoustic_msgs` dep added.

**Verification**: 124 package tests, 0 failures (9 new geometry gtests). cpplint clean; uncrustify clean. Probe on `bizzyboat_sonar/2026-06-18T19-39-06+00-00`: 48 551 pings (port/stbd/down balanced), 676 m track, earth→bizzy/map available, ~24% pings no-TF (early-bag warm-up, expected).

### Open questions
- [ ] PR2 must tune nadir altitude auto-detect (probe showed implausible ~0.1 m — near-field ringing beats min_gate=1; safe-degrades to flat). Visually verifiable once rendering lands.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-22
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved (all must-fixes addressed in-branch)

**Branch**: feature/issue-7 at `072ea19`
**Mode**: pre-push
**Depth**: Standard (reason: ~770 LOC new subsystem, cross-layer ROS/TF)
**Must-fix**: 2 → fixed | **Suggestions**: several (key ones fixed)
**Round**: 1 | **Ship**: recommended — no open must-fix after fixes; tests green
**Specialists**: static (clean, pre-run), governance + plan-drift (lead), 2 Claude adversarial passes (Lens A logic, Lens B systemic). Copilot off (default + quota suspended).

### Findings
- [x] (must-fix) `is_bigendian` ignored → silent garbage on big-endian bag; byte-swap added — `sidescan_bag_session.cpp:normalize_beam0`
- [x] (must-fix) one corrupt message aborted whole load; per-message try/catch + decodeErrors() — `sidescan_bag_session.cpp` pass1/pass2
- [x] (suggestion) nearest-nadir altitude had no staleness cap; added altitude_max_dt_s=2s — `sidescan_bag_session.cpp`
- [x] (suggestion) ground_range NaN/Inf guard + 2 gtests — `sidescan_geometry.hpp`
- [x] (verify) sample_rate top-level + beam-major layout — confirmed correct against .msg, no change
- [ ] (suggestion, → PR2) unbounded in-RAM ping index (distance-buffer milestone); single-stamp canTransform geo signal; amplitude-estimator min_gate (fallback only)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-23 06:38 -0400
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: changes-requested
**Branch**: feature/issue-7 at `3d1c1fd`
**Mode**: pre-push
**Depth**: Deep (reason: 3812-line new Qt app + core + tests, concurrency + geometry)
**Must-fix**: 2 | **Suggestions**: 6
**Round**: 2 | **Ship**: continue — two confirmed must-fix on the render/error path

### Findings
- [ ] (must-fix) Render worker unguarded: reader.open throw in readWindow crosses QtConcurrent boundary -> result() rethrows on GUI thread -> terminate; onRenderFinished also ignores r.ok — `sidescan_bag_session.cpp:520` / `sidescan_viewer_window.cpp:609,616`
- [ ] (must-fix) Stale render applied with no session/generation token -> old-bag flash + waterfall index on wrong geometry (mislocated mark) — `sidescan_viewer_window.cpp:614`
- [ ] (suggestion) No destructor/closeEvent waitForFinished() on watchers (defensive quit-during-load) — `sidescan_viewer_window.cpp`
- [ ] (suggestion) readWindow slot map: two same-channel pings sharing header stamp_ns overwrite slot, one ping silently dropped — `sidescan_bag_session.cpp:506`
- [ ] (suggestion) readWindow break keys on recv_timestamp vs header-stamp window+3s pad; clock skew can drop in-range ping — `sidescan_bag_session.cpp:528`
- [ ] (suggestion) make_box_contact negative stamp_s -> nanosec wrap; clamp defensively — `contact_store.cpp:36`
- [ ] (suggestion) package.xml description/exports omit new executables; unused heavy tuner deps (OpenCV/cv_bridge/ffmpeg/grid_map_core) — `package.xml`
- [ ] (suggestion) interp_base_pose dead it==begin() branch bypasses max_gap guard if line-85 check loosened — `sidescan_bag_session.cpp:86`
