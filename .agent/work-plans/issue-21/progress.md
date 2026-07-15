---
issue: 21
---

# Issue #21 — Survey explorer stage 3: multi-pass MBES pointcloud drill-down

## Issue Review
**Status**: complete
**When**: 2026-07-15 17:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #21
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Multi-select + 3D view action is a transparent, user-driven operation. No hidden automation or invisible state changes. |
| Only what's needed | OK | Explicit out-of-scope items (CUBE generation, sidescan draping, contacts overlay) keep scope tight. Use case is concrete: comparing multi-pass MBES evidence for the same area. |
| Improve incrementally | OK | Natural stage-3 increment on merged stage 2 (PR #20). Four bounded scope items; single PR is feasible. |
| Capture decisions, not just implementations | Watch | Two design decisions will arise that warrant capture: (1) how windowed reads determine time intervals from pass metadata (seek-to-start vs. full-bag scan); (2) how color coding handles palettes for N passes (fixed vs. dynamic). Plan phase should record the chosen approach. |
| A change includes its consequences | Watch | No tests are mentioned in the issue. Existing `test/` directory suggests tests are expected. Plan should specify what will be tested. |
| Test what breaks | Watch | Multi-pass windowed reads and cross-pass georeferencing are regression-prone. The plan should name test targets for the windowed read path and coordinate-frame consistency. |
| Workspace vs. project separation | OK | Correctly scoped to the `marine_perception_tools` project repo. No workspace contamination risk. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0001 (Adopt ADRs) | Yes — Watch | Windowed-read implementation strategy and multi-pass georeferencing approach are design decisions. Capture in the plan or a brief ADR; "evaluate during planning" in the issue is acceptable deferral. |
| ADR-0002 (Worktree isolation) | Yes — OK | Worktree `issue-marine_perception_tools-21` already exists. |
| ADR-0008 (ROS 2 conventions) | Watch | If the implementation introduces new ROS 2 interface types or TF lookups, they must follow Rolling conventions. The issue's reference to "the same transformation framework as the indexer" suggests TF reuse — plan should confirm TF2 is used consistently. |
| ADR-0013 (progress.md vocabulary) | Yes — OK | This entry uses `## Issue Review`. |

### Consequences

- New windowed bag-read path may extend `bag_loader.cpp` — related tests in `test/` should be reviewed and extended in the same PR.
- `point_cloud_view.cpp/hpp` is the identified reuse candidate; if its interface is extended for multi-pass input, any callers of the current single-pass interface need updating in the same PR.
- README notes on survey explorer capabilities should be updated to reflect stage 3 once implemented.

### Actions
- [ ] Plan phase: document the windowed-read implementation strategy (seek vs. full-scan) and color-palette approach as explicit design decisions.
- [ ] Plan phase: specify test coverage for multi-pass windowed reads and georeferencing correctness.
- [ ] Implementation: confirm TF2 is used for georeferencing, consistent with the indexer (ADR-0008).
- [ ] Implementation: update README and any existing single-pass callers of `point_cloud_view` if its interface changes.

## Plan Authored
**Status**: complete
**When**: 2026-07-15 18:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-21/plan.md` at `bd1a152`
**Branch**: feature/issue-21 at `bd1a152`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-15 18:01 +00:00
**By**: Claude Code Agent (Claude Opus)
<!-- Independent review. The name-based self-review heuristic in review-plan
     collides here (all workspace agents share the name "Claude Code Agent"),
     but this is a fresh-context Opus sub-agent dispatched per the #490 handoff
     contract, distinct from the Sonnet author of ## Plan Authored — so no
     author-self-review annotation. -->

**Plan**: `.agent/work-plans/issue-21/plan.md` at `bd1a152`
**PR**: PR-less (--issue mode)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (suggestion) `mbes_bag_reader.cpp` is Qt-free bag ingest — belongs in the `sidescan_core` library (alongside `sidescan_bag_session.cpp`), not in the `sidescan_target_viewer` executable sources as step 6 states. Matches the established Qt-free-ingest split and lets `test_mbes_bag_reader` link `sidescan_core` (like `test_coverage_raster`) instead of recompiling the source. — `plan.md:97`
- [ ] (suggestion) Windowed-read design decision cites the wrong precedent: `SidescanBagSession::readMbesWindow` (the directly analogous code) already uses `Reader::seek()` with a try/catch fallback to sequential scan (`sidescan_bag_session.cpp:940`); the plan justifies a pure linear scan via `bag_loader.hpp`'s note (a different consumer). Adopt the proven seek+fallback pattern. — `plan.md:41`
- [ ] (suggestion) Reuse the world-lift helpers `rotate_by_quat` / `lookup_at_or_latest` (anon-namespace in `sidescan_bag_session.cpp`) — they do exactly the TF capture + sensor→world lift the new reader needs. Both would live in `sidescan_core`; extract/share rather than re-implement ("Only what's needed"). — `plan.md:36`
- [ ] (suggestion) Test approach diverges from the package norm: no existing test writes a synthetic rosbag; bag-touching logic is covered by pure-math tests (`test_mbes_geometry`). A synthetic-bag test exercises framework glue — mild tension with the plan's own "test what breaks, not the framework glue" note. Consider factoring the pure world-lift (given a captured transform) into a bag-free testable function. — `plan.md:88`
- [ ] (suggestion) Scope-count inconsistency: "Files to Change" lists 12 rows but "Estimated Scope" says "~8 files." At 12 files / 4 components the PR is at the upper single-PR bound, but the changes form one cohesive vertical slice (reader → color mode → window → wiring) — keep together; just fix the count. — `plan.md:156`

## Implementation
**Status**: complete
**When**: 2026-07-15 14:59 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**Branch**: feature/issue-21 at `98bd095`

Implemented per the amended plan (all 5 review-plan suggestions folded in
at `23ee2fd`): `read_mbes_window` in sidescan_core (seek + topic-filter,
tf_lift.hpp helpers shared with the session), cross-bag earth-anchor
reprojection into the first pass's frame, PointCloudView ColorMode::Pass
+ setMultiPassPoints (golden-angle pass_color), MbesCloudWindow
(QtConcurrent load, legend, visible skip counts), overview
extended-select + "View MBES cloud" button, `sidescan_probe
--mbes-window` headless check, .agents/README.md row updated (top-level
README has no viewer section — the inventory row is the doc home).

Deviations from plan, both found by the real-data smoke:
- /tf is SEEKED (15 s pre-window history), not scanned from bag start:
  the earth<-map anchor is continuously republished in these bags
  (verified by sampling /tf mid-bag), and a start-scan costs time
  proportional to the window's position.
- Removed an accidental per-ping exact `reserve()` that made the
  sounding append quadratic — 50 s -> 478 ms for a ~1M-sounding pass
  (3,921 pings, matching the index's count exactly; geo anchor
  captured; 0 skipped).

Verified: 311 tests / 0 failures (was 274); overview offscreen smoke;
real-bag windowed reads at two positions in a 2.5 GB Massabesic bag.
