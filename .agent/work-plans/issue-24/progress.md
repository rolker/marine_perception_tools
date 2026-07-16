---
issue: 24
---

# Issue #24 — Integrated explorer shell: single main window, tile-granular map selection, nav track, rename

## Issue Review
**Status**: complete
**When**: 2026-07-16 14:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #24
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Human control and transparency | OK | Single main window is more transparent than multiple top-level windows; docking approach preserves per-view configurability. Selection signals are explicit user actions. |
| Only what's needed | OK | The restructure is motivated by a concrete need (stage 4 CUBE lab needs a proper home; multiple one-off windows are acknowledged as scaffolding). Scope is bounded by 4 explicitly listed items; out-of-scope is implicit from the stage-4 framing. |
| Improve incrementally | Watch | Four items in one PR is on the larger side, but they form a coherent restructure that must land together before stage 4. Item 3 (nav track) has an open dependency; the plan should sequence it as a gate or split it into a follow-up. |
| Capture decisions, not just implementations | Watch | The rename rationale (sidescan_target_viewer → survey-explorer) and the QDockWidget re-parenting strategy (re-parent vs. rewrite) are design decisions worth capturing in the plan. The issue body notes the rename closes the timing question from #22 — the plan should make this explicit. |
| A change includes its consequences | Action needed | The rename has downstream doc/config consequences: `.agents/README.md` (Package Inventory, CLI examples, layout table), `README.md` (CLI usage, future-work link #1), `CMakeLists.txt` (executable name), and `package.xml` (if exec is registered). All must be updated in the same PR. The sidescan_target_viewer shim entry point needs to appear in `CMakeLists.txt` explicitly. |
| Test what breaks | Watch | QDockWidget re-parenting of existing windows needs testing. The existing test pattern (offscreen GL smoke test for `test_point_cloud_view`) shows the project already tests Qt widgets offscreen. The plan should name what tests cover the re-parented docks and the tile-selection interaction path. |
| Workspace vs. project separation | OK | Correctly scoped to the `marine_perception_tools` project repo. No workspace contamination risk. |
| Primary framework first, portability where free | OK | Qt5/ROS 2 conventions; CMake version-agnostic Qt targets already in place. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0001 (Adopt ADRs) | Watch | Re-parenting strategy and rename decision are design choices. Capture in the plan is sufficient; no new ADR is required unless the naming convention deviates from package conventions. |
| ADR-0002 (Worktree isolation) | OK | Worktree `issue-marine_perception_tools-24` already exists and is on `feature/issue-24`. |
| ADR-0008 (ROS 2 conventions) | Watch | Renaming the executable requires checking `CMakeLists.txt` install rules and `package.xml` exec-depend declarations for any registered entry points. ROS 2 convention: executables are listed under `<exec_depend>` only if they call rosdep-managed libraries, not for local executables — but the install rule must be present and correct. |
| ADR-0013 (progress.md vocabulary) | OK | This entry uses `## Issue Review`. |

### Consequences

Per the consequences map:
- **Rename** (`sidescan_target_viewer` → `survey_explorer` or similar): update `CMakeLists.txt` install rule, `.agents/README.md` Package Inventory + CLI examples + layout table, `README.md` CLI usage section, and any CI step that names the executable explicitly.
- **QDockWidget re-parenting** of `MbesCloudWindow` and `SidescanViewerWindow` into docks: their existing window-class APIs may need minor interface changes (parent widget, visibility toggling). Test fixtures for those windows must be updated in the same PR.
- **Tile-granular selection** replaces point-click: `SurveyOverviewCanvas` interaction model changes; any test that simulates a click-based pass selection must be updated.

### Recommendations

- Gate item 3 (nav track overlay) explicitly in the plan: either as a follow-up issue contingent on `unh_marine_autonomy#265` landing, or as a guarded block (`if track table present`) that degrades gracefully. The issue already notes graceful degradation — the plan should confirm which approach and add a test for the degrade path.
- Enumerate the rename consequence list (above) explicitly in the plan checklist so it isn't missed during implementation.
- Confirm whether `sidescan_target_viewer` remains as a binary shim (`main()` that calls the new entry point) or is removed. A shim keeps backward compatibility for any scripted callers; the issue says "remains as a shim entry point" — make this explicit in CMakeLists.txt.

### Actions
- [ ] Enumerate rename consequences in the plan (CMakeLists.txt, .agents/README.md, README.md, package.xml) and confirm shim strategy.
- [ ] Gate or split item 3 (nav track) on `unh_marine_autonomy#265` dependency.
- [ ] Name test targets for QDockWidget re-parenting and tile-selection interaction in the plan.
