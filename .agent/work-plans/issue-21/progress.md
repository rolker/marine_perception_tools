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
