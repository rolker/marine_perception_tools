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

## Plan Authored
**Status**: complete
**When**: 2026-07-16 15:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-24/plan.md` at `1a0b90d`
**Branch**: feature/issue-24 at `1a0b90d`
**Phases**: single

### Open questions
- [ ] Rename target name: plan proposes `survey_explorer` — needs Roland's confirmation before implementation begins.
- [ ] Dock strategy for `SidescanViewerWindow`: Option A (wrap QMainWindow in QDockWidget) vs Option B (refactor to QWidget) — confirm acceptable before step 2.

## Plan Review
**Status**: complete
**When**: 2026-07-16 18:10 +00:00
**By**: Claude Code Agent (Claude Opus)

<!-- Independence: `## Plan Authored` was by "Claude Code Agent (Claude Sonnet)"; this
review is a fresh-context dispatch on a different model (Opus). All workspace agents
share the name "Claude Code Agent", so the name-only self-review heuristic over-matches
here — treated as an independent review per the annotation's stated purpose. -->

**Plan**: `.agent/work-plans/issue-24/plan.md` at `1a0b90d`
**PR**: PR-less (`--issue` mode; gh unauthenticated this session — issue requirements
read from the `## Issue Review` entry above and the plan's Context/Issue links)
**Verdict**: approve-with-suggestions

### Findings
- [ ] (must-fix) `.agents/README.md` omitted from the Files-to-Change table — its Package Inventory row and CLI examples name `sidescan_target_viewer` as the tool; review-issue explicitly listed it. Update to name `survey_explorer` primary + note the shim — `plan.md:68` (Files table), `plan.md:63`
- [ ] (must-fix) New dock smoke test `test_survey_explorer_window.cpp` is named in the Principles table but absent from the Files-to-Change table and the CMakeLists change description (no `ament_add_gtest` registration listed). Reconcile: add the file + register the target, or drop the claim — `plan.md:90`, `plan.md:77`
- [ ] (suggestion) `queryAllNavTrack()` uses raw SQL against `nav_track`, contradicting the bridge's stated contract ("reusing marine_survey_index's query library … not by re-implementing the SQL", `survey_index_bridge.hpp:39`). The library exposes `queryNavTrack(bag_id)` and `queryNavTrackInBox(...)`; prefer reusing them (loop distinct bag_ids, or box=index extent) or add a true all-bags accessor to the library — `plan.md:54`
- [ ] (suggestion) `queryTileBox(south,west,north,east)` takes a box, but the plan describes it as "expands the union of selected-tile bounds"; the union must be computed at the window/canvas (owner of `tiles_`) from the emitted indices, not in the bridge. Clarify the boundary — `plan.md:48`
- [ ] (note) Two open-question checkpoints (rename name; dock strategy A vs B) correctly gate steps 1 and 2 — get Roland's confirmation before those steps — `plan.md:112`

### Evaluation
| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | ~8–10 files, 2 new sources, single PR. Upper bound but coherent — docking depends on the rename; the tile-selection signal change is the same signal the dock connection reads. |
| Issue alignment | Good | All four issue items covered. review-issue's item-3 (nav track) gate is resolved: `unh_marine_autonomy#265` has landed (`queryNavTrack`/`queryNavTrackInBox` present, prod index holds v2 data), and graceful degradation + empty-table test are planned. |
| File targeting | Needs work | Right source files identified, but `.agents/README.md` (a review-issue-flagged consequence) is missing and the named dock smoke-test file is not listed. |
| Consequences | Needs work | Consequences table covers the rename's `CMakeLists`/`package.xml`/`README.md`, but omits `.agents/README.md` (Package Inventory + CLI + layout table). |
| Principle alignment | Good | "A change includes its consequences" mostly satisfied (gap = `.agents/README.md`); "Only what's needed" respected (reuses query API, no new schema); shim preserves backward compat. |
| ADR compliance | Good | ADR-0008 install rule already correct (`lib/${PROJECT_NAME}`); no new `exec_depend` needed for a local exe; ADR-0013 entry types correct. No new ADR required for the snake_case rename. |
| ROS conventions | Good | Executable rename + install(TARGETS) + package.xml handled per ROS 2 conventions; Qt5-version-agnostic targets preserved. |

### Summary
Structurally sound, well-reasoned plan that addresses most review-issue findings and resolves the nav-track dependency gate. Two completeness gaps remain — the `.agents/README.md` rename consequence and the unlisted dock smoke-test target — both additive (they don't change the approach) and should be folded into the plan before/during implementation. Ready to implement once those are folded in and the two checkpoint questions are confirmed with Roland.

### Recommended Actions
- [ ] Add `.agents/README.md` to the Files-to-Change + Consequences tables (Package Inventory, CLI examples, layout).
- [ ] List `test_survey_explorer_window.cpp` in the Files table and add its `ament_add_gtest` registration to the CMakeLists change, or drop the claim.
- [ ] Decide `queryAllNavTrack` raw-SQL vs query-library reuse; document the choice in the plan.
- [ ] Confirm the rename target name and the dock strategy (Option A/B) with Roland before steps 1–2.

## Plan Superseded and Rewritten
**Status**: complete
**When**: 2026-07-16 (host, at plan checkpoint)
**By**: Claude Code Agent (Claude Fable 5)

Roland corrected the architecture at the checkpoint: no shell window hosting
QDockWidgets — SidescanViewerWindow already has the right panes; grow IT into
the explorer. New flow: geo pane = index map (stores basemap + nav track +
tile selection + current-pass drape + contacts, one merged canvas); tile
selection auto-loads ALL bags' passes into the cloud (stage-4 CUBE surface
joins that display later); a NEW timeline pane (gap-compressed UTC axis,
pass bars sharing the cloud's golden-angle colours) picks what the
single-pass waterfall shows. SurveyOverviewWindow/Canvas + MbesCloudWindow
retire. The first review-plan round reviewed the superseded draft — a fresh
review-plan round runs against this rewrite. Open checkpoint items: executable
name (survey_explorer proposed); timeline replaces the pass list (flag if the
tabular list should be retained).

## Plan Review
**Status**: complete
**When**: 2026-07-16 18:27 +00:00
**By**: Claude Code Agent (Claude Opus)

<!-- Independence: the most recent `## Plan Authored` was by "Claude Code Agent
(Claude Sonnet)" and the rewrite by "Claude Code Agent (Claude Fable 5)". This
review is a fresh-context dispatch on a different model (Opus). All workspace
agents share the name "Claude Code Agent", so the name-only self-review heuristic
over-matches — treated as independent per the annotation's stated purpose. -->

**Plan**: `.agent/work-plans/issue-24/plan.md` at `b854d81` (rewrite per checkpoint correction)
**PR**: PR-less (`--issue` mode; gh unauthenticated this session — issue requirements
read from the `## Issue Review` entry above and the plan's Context/Issue links)
**Verdict**: approve-with-suggestions

This is a fresh review of the **rewritten** plan (the prior `## Plan Review`
graded the superseded QDockWidget-shell draft).

### Findings
- [ ] (must-fix) Step 1 asserts `SidescanCanvas` "already carries the geo projection" — it does NOT. `SidescanCanvas` is a per-bag map-ENU (metres) canvas (`sidescan_canvas.hpp:41`); the geographic `GeoView`/`geoToPixel` projection lives in `survey_overview_projection.hpp`, used by `SurveyOverviewCanvas` (which states the split explicitly at `survey_overview_canvas.hpp:41`). The merge means the index map adopts a geographic frame and the current-pass coverage is reprojected map-ENU→geo (anchor exists: `SidescanBagSession::mapToGeo`, used at `sidescan_viewer_window.cpp:981`). Correct the claim and add the reprojection step — `plan.md:36`, `plan.md:44`
- [ ] (must-fix) The merged canvas needs the pure `survey_overview_projection.hpp` (already unit-tested by `test_survey_projection.cpp`), but the Files-to-Change table deletes `survey_overview_canvas.*` while staying silent on the projection header. State it is retained + reused (and its test kept) so the geographic math isn't reinvented — `plan.md:100`, `plan.md:101`
- [ ] (suggestion) `queryAllNavTrack()` implementation unstated; the bridge contract is to reuse `marine_survey_index`'s query library, not raw SQL (`survey_index_bridge.hpp:39`). `queryNavTrackInBox(extent())` (bridge already has `extent()`) gives all-bags nav track with no new SQL — document the choice (carried from prior round, still open) — `plan.md:54`
- [ ] (suggestion) Scope at upper bound (~16 files); the `PassTimelineWidget` (step 4) is the most self-contained additive piece and a natural follow-up split if the PR balloons — gate phase (e) as spillable — `plan.md:70`
- [ ] (note) `queryTiles(vector<GridIndex>)` wraps `queryPasses` with tiles as direct index keys — resolves the prior round's box-detour concern — `plan.md:57`
- [ ] (note) Integrated-window smoke: rewrite folds the prior dock-smoke concern into "existing window/GL tests … offscreen smoke of the integrated window (`--index` mode)" without naming a file — ensure the `--index` path gets an actual offscreen smoke assertion — `plan.md:109`
- [ ] (note) Two checkpoint questions correctly gate work: executable name (`survey_explorer`) and timeline-replaces-pass-list — confirm with Roland before steps 1/4 — `plan.md:132`

### Summary
Sound, technically feasible rewrite — the multi-pass cloud API (`PointCloudView::setMultiPassPoints`) and the per-bag `mapToGeo` anchor both already exist, and "grow the viewer" is cleaner than the superseded shell. The one substantive gap: step 1 mis-describes which component holds the geographic projection, so the canvas merge is really "adopt a geographic frame and reproject the per-bag coverage," reusing `survey_overview_projection.hpp`. Fold findings 1–3 into the plan (all additive) and confirm the two checkpoint questions, then ready to implement.

## Plan Checkpoint
**Status**: complete
**When**: 2026-07-16
**By**: Roland (decisions) / Claude Code Agent (Claude Fable 5)

Roland approved the rewritten plan with review-plan round-2 findings folded in.
Decisions: executable name = `survey_explorer`; timeline REPLACES the pass
list (timeline only). Implementation proceeds host-inline, commit-phased
(a) bridge+pure headers+tests, (b) canvas merge, (c) cloud in place,
(d) timeline, (e) rename/docs/retire.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-22 11:45 -0400
**By**: Claude Code Agent (Claude Fable 5)
**Verdict**: approved

**Branch**: feature/issue-24 at `f9fae4d`
**Mode**: pre-push
**Depth**: Deep (reason: 33 files, +3301/-1150; GUI concurrency + executable rename)
**Must-fix**: 1 | **Suggestions**: 5
**Round**: 1 | **Ship**: recommended — the single must-fix and all actionable suggestions were fixed in-session (f9fae4d); 377 tests green after fixes.

Specialists: Static Analysis (via full ament lint suite in colcon test, green),
Governance, Plan Drift, Claude Adversarial Lens A + Lens B (Deep). Copilot: off (default).

### Findings
- [x] (must-fix) UAF: superseded background indexer captures `this`, never waited — orphaned futures now tracked + waited in dtor — `src/sidescan_viewer_window.cpp` (openBag/dtor)
- [x] (suggestion) Timeline short bars drawn 3 px but hit-tested in fraction space → dead click targets; now hit-tests drawn rects — `src/pass_timeline_widget.cpp`
- [x] (suggestion) Shim no-/proc fallback could exec CWD-relative name; now fails loudly when self-path unresolvable — `src/sidescan_target_viewer_shim.cpp`
- [x] (suggestion) Antimeridian restriction of equirectangular canvas undocumented; note added — `src/sidescan_canvas.cpp` (geoToCanvas)
- [x] (suggestion) Stale phase-e staging comment in CMake test block; dropped — `CMakeLists.txt`
- [x] (suggestion) pass_coalesce.hpp missing from plan Files-to-Change table; row added — `.agent/work-plans/issue-24/plan.md`
- [ ] (note, out of scope) Repo lacks ADR-0017 root AGENTS.md — pre-existing gap, separate onboarding task

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-22 17:17 -0400
**By**: Claude Code Agent (Claude Fable 5)
**Verdict**: approved

**Branch**: feature/issue-24 at `6164ca8`
**Mode**: pre-push
**Depth**: Deep (reason: 16 desk-round commits since round 1 — time bar, cache, perf, GPU bounds)
**Must-fix**: 0 | **Suggestions**: 8 (all applied)
**Round**: 2 | **Ship**: recommended — zero must-fix from both adversarial lenses; all suggestions fixed in-session; 431 tests green.

Specialists: Claude Adversarial Lens A + Lens B (Deep, fresh-context), Governance+Plan-Drift; static analysis via full ament lint suite (green). Copilot: off.

### Findings
- [x] (suggestion) Docs behind desk rounds: basemap controls, legend checkboxes, contact clip/deletion — README/.agents/README updated; summarizing plan note added
- [x] (suggestion) Waterfall 4000-row cap was silent — now in status line
- [x] (suggestion) Index worker uncaught exception would std::terminate (Qt5) — try/catch + tightened cache count guards
- [x] (suggestion) Shared .tmp cache path raced across processes — per-writer pid suffix
- [x] (suggestion) panning_ could stick on lost release — self-heals at paint
- [x] (suggestion) Zero-size layer-cache pixmap guard
- [x] (suggestion) const ticket defeated the pass-clouds move (full copy on UI thread) — non-const now
- [x] (suggestion) monthName comment inaccuracy
- [x] (note) GeoZui4D port attribution assumes personal (not CCOM-institutional) copyright — Roland to confirm; his call as author (resolved: operator confirmed **personal**; attribution added to ported time-bar file headers, `62f63c2`)

## Integrated Review
**Status**: complete
**When**: 2026-07-27 15:20 -04:00
**By**: Claude Code Agent (Claude Opus)

**PR**: #25 at `59d8ba8`
**Sources**: 3 (Copilot R1 @ `59d8ba8`, Local Review (Pre-Push) R1 @ `f9fae4d` + R2 @ `6164ca8`, CI rollup)
**Cross-source confirmations**: 0
**CI**: all-pass (build-and-test success, copilot-pull-request-reviewer success)

### Findings
- [x] (must-fix, Copilot) Cached ping channel is cast to `SidescanChannel` without range validation; a readable-but-corrupt cache can yield channel >= 3, which is then used to index a 3-element `std::array` (`sidescan_bag_session.cpp:822` slot write, `:852` read) — out-of-bounds write, not merely bad enum data. Validate `channel` in [0, kNumSidescanChannels) in `getPing()` and return false so `loadSessionIndex` rejects the cache and the bag re-indexes — `src/session_index_io.cpp:90-109`
- [x] (suggestion, Copilot) `saveSessionIndex()` leaves the per-writer temp file (`*.tmp.<pid>`) behind on the post-write failure return; only the rename-failure path removes it. Stale temps accumulate in the cache dir over repeated failures (full disk is the likely trigger, and it makes the disk-full state worse). Remove `tmp` before returning false at the write-failure path — `src/session_index_io.cpp:237`
- [x] (suggestion, Copilot) `TimeBarWidget::clearPasses()` also zeroes the extent and `user_adjusted_`, which the header's `setExtent` comment presents as pass-independent state. The sole caller (`exitSelectionCloud`) deliberately re-applies extent and visibility right after, so no live defect — but the undocumented coupling is a trap for the next caller. Document the reset in the `clearPasses()` header comment (behavior is intended; the doc is what is missing) — `src/time_bar_widget.hpp:73`

### False positives
- (none) All three Copilot comments describe real code properties; two are latent rather than live, but per the Quality Standard validation/silent-failure concerns are not dismissed as nits.

### Notes
- Local review R2 finding "shared .tmp cache path raced across processes" touches the same function as the Copilot temp-leak finding but is a distinct concern (race vs. leak) — not counted as a cross-source confirmation.
- Carried forward, unresolved from Local Review R2: GeoZui4D port attribution (personal vs. CCOM-institutional copyright) awaits Roland's call — not an agent-actionable item.
- **Operator decisions (2026-07-27 checkpoint, Roland)**: (1) address all three findings above in one fix pass; (2) GeoZui4D port attribution is **personal** (credit the original author personally in the ported files' copyright headers) — apply the header attribution in the same pass and check off the carried-forward note in the Local Review R2 entry.

## Implementation
**Status**: complete
**When**: 2026-07-27 19:37 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-24 at `62f63c2`
**Addressed**: Integrated Review (When 2026-07-27 15:20 -04:00, PR #25 @ `59d8ba8`) + operator decisions in that entry's Notes
**Commits**: `80c8bc0`, `964de87`, `79ab657`, `62f63c2`

### Actions
- [x] (must-fix) Validate cached sidescan channel range in `getPing()`; reject out-of-range cache so the bag re-indexes (prevents OOB array index) — `src/session_index_io.cpp:102` — `80c8bc0`
- [x] (suggestion) Remove the per-writer temp file on the `saveSessionIndex()` write-failure path, not just the rename-failure path — `src/session_index_io.cpp:237` — `964de87`
- [x] (suggestion) Document that `clearPasses()` also resets the extent + `user_adjusted_` state — `src/time_bar_widget.hpp:73` — `79ab657`
- [x] (operator decision) GeoZui4D TimeControl port attributed to the original author's **personal** copyright in the ported time-bar headers — `src/time_bar_model.hpp`, `src/time_bar_widget.hpp`, `src/time_bar_widget.cpp` — `62f63c2` (also checked off the carried-forward note in Local Review R2)

### Notes
- Deferred / carried forward (not agent-actionable): the Local Review R2 "GeoZui4D port attribution" question is now resolved by the operator decision above and checked off in that entry.
- Build not run: the worktree's lower layers (underlay/core/…/simulation `install/`) are unbuilt, so `marine_autonomy` (a `find_package` dep) is unresolvable and the package cannot compile here — a pre-existing environment gap, unrelated to #24. Changes verified by inspection: the channel check is an `int32_t` range compare against `kNumSidescanChannels` (in scope via the header chain); the temp cleanup is the same `std::filesystem::remove(tmp, ec)` call already used on the rename-failure path; the remaining edits are comment-only. Pre-commit hooks (lint) passed on every commit.

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-27 19:43 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: approved

**Branch**: feature/issue-24 at `62f63c2` (code) / `371e954` (progress tip)
**Mode**: pre-push (gh unauthenticated + code remote is `gitcloud`; reviewed vs local `origin/jazzy`)
**Depth**: Deep-context, delta-scoped (reason: round-3 verification of the 20-line Integrated-Review fix delta since R2 `6164ca8`; full branch is Deep and twice-reviewed at R1/R2)
**Must-fix**: 0 | **Suggestions**: 0
**Round**: 3 | **Ship**: recommended — zero must-fix; the delta correctly closes all three Integrated-Review findings plus the GeoZui4D attribution decision, verified by inspection, an independent fresh-context adversarial pass (no findings), and cppcheck (clean on touched lines).

Specialists: Static Analysis (cppcheck on `session_index_io.cpp`, the only code-bearing delta file — clean on touched lines; sole hit is a pre-existing `useStlAlgorithm` style nit on an untouched context line in `sidescan_geometry.hpp`). Claude Adversarial (one fresh-context combined-lens pass on the delta — no findings). Copilot: off (default). Local: not run (delta-scoped round-3 verification). Governance/Plan-Drift: delta introduces no new consequences or scope; plan reconciled in R1/R2.

### Findings
- [ ] No issues found. LGTM.

### Verified fixes (delta since R2 `6164ca8`)
- [x] (must-fix, closed) Cached sidescan channel range-validated in `getPing()`; out-of-range → `false` → `loadSessionIndex` returns `nullopt` → bag re-indexes (prevents OOB `kNumSidescanChannels`-element array index) — `src/session_index_io.cpp:108`
- [x] (suggestion, closed) Per-writer temp file removed on the `saveSessionIndex()` write-failure path (mirrors the rename-failure `remove`; `ec` in scope) — `src/session_index_io.cpp:243`
- [x] (suggestion, closed) `clearPasses()` doc documents the extent + `user_adjusted_` reset; matches the implementation and the `exitSelectionCloud` re-apply — `src/time_bar_widget.hpp:76`
- [x] (operator decision, closed) GeoZui4D TimeControl port attributed to the author's personal copyright in the 3 ported time-bar files — `src/time_bar_model.hpp`, `src/time_bar_widget.hpp`, `src/time_bar_widget.cpp`

## Integrated Review
**Status**: complete
**When**: 2026-07-31 11:07 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #25 at `0e8fb0d`
**Sources**: 3 (Copilot R2 @ `4ee70d8`, Copilot R3 @ `0e8fb0d`, CI rollup; Copilot R1 @ `59d8ba8` fully triaged/fixed in the 2026-07-27 round)
**Cross-source confirmations**: 0
**CI**: all-pass (build-and-test green on `0e8fb0d`)

### Findings
- [x] (suggestion, Copilot R2) `pageBy()` computes the page delta as `direction * width() * spp_ * 1e9` in double and casts to `int64_t`; at `kMaxSpp` (2.7e6, reachable via zoom-out clamp at line 457) with a ~4K-wide widget the product (~1.1e19) exceeds `INT64_MAX` → out-of-range float→int cast is UB. Fix: compute delta in double and clamp before casting (shared helper) — `src/time_bar_widget.cpp:588`
- [x] (suggestion, local sweep — same root cause) identical unclamped `double → int64_t` ns casts in `timeOfX()` (`x_px - width()/2` up to ±width) and the drag-pan branch (`dx` up to ±width); fix with the same clamped helper — `src/time_bar_widget.cpp:210`, `src/time_bar_widget.cpp:459`
- [x] (suggestion, Copilot R3) `indexedTiles()` casts `tile_row`/`tile_col` straight to `uint32_t`; a negative out-of-contract row wraps to ~4e9 and yields nonsense bounds/keys. `level` is already guarded — guard `row/col < 0 → continue` before converting, consistent with it — `src/survey_index_bridge.cpp:87-88`
- [x] (suggestion, Copilot R3) `computeTickLadder()` uses truncating `/` and signed `%` on `left_ns`; for pre-epoch left edges (reachable: full zoom-out at `kMaxSpp` puts the left edge ~171 years before center, or plain leftward pan — center/pan are unclamped) `frac` goes negative and `whole_s` is off by one → misplaced ticks/labels. Normalize the remainder into [0, 1e9) — `src/time_bar_model.hpp:150-151`

### False positives
- (none)

## Implementation
**Status**: complete
**When**: 2026-07-31 11:32 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**Branch**: feature/issue-24 at `dd7b797`

All four findings from the 2026-07-31 Integrated Review addressed, one
atomic commit each:

- `785503b` — saturating `offsetTimeNs()` in `time_bar_model.hpp` replaces
  the raw `double -> int64` ns casts in `pageBy()`, `timeOfX()`, and the
  drag-pan branch (covers the additive overflow as well as the cast UB);
  unit test pins exactness in range + saturation both directions.
- `cc730b8` — `computeTickLadder()` floor-divides `left_ns` so pre-epoch
  window edges keep tick phase; test pins the minute tick at 0.5 px for a
  left edge 0.5 s before the epoch.
- `dd7b797` — `indexedTiles()` skips rows with negative `tile_row`/
  `tile_col` (matching the existing level guard); fixture test injects
  negative row, negative col, and unknown-level rows.

Verified: package rebuild clean; 434 tests, 0 failures (62 skipped =
display-gated); cpplint/uncrustify/copyright/cppcheck/lint_cmake all clean.

## Integrated Review
**Status**: complete
**When**: 2026-07-31 11:58 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #25 at `50e4370`
**Sources**: 2 (Copilot re-review @ `50e4370`, CI rollup)
**Cross-source confirmations**: 0
**CI**: all-pass (build-and-test green 9m17s on `50e4370`)

Copilot's re-review generated no new comments; its one SUPPRESSED
(low-confidence) comment is a valid catch — the fourth member of the
overflow family fixed in `785503b`, missed by the local sweep because it
multiplies by a precomputed ns/px factor rather than `spp_ * 1e9`.

### Findings
- [x] (suggestion, Copilot suppressed) scrollbar-thumb drag computes
  `center_ns_` via an unbounded `double -> int64` cast
  (`thumb_start_center_ + (dx_px) * per_px`); routed through
  `offsetTimeNs()` like the other three interactions — `src/time_bar_widget.cpp:470`

### False positives
- (none; the two inline comments at this head are the previously fixed
  R1/R2 threads re-anchored by GitHub, not new findings)

## Integrated Review
**Status**: complete
**When**: 2026-07-31 12:20 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #25 at `7040499`
**Sources**: 2 (Copilot re-review @ `7040499`, CI rollup)
**Cross-source confirmations**: 0
**CI**: all-pass (build-and-test green 8m54s on `7040499`)

Copilot again generated no new inline comments; both SUPPRESSED
(low-confidence) comments were valid members of the same conversion
family and are fixed, plus one more found by a local truncation/cast
sweep of the touched files (sweep is now clean — no truncating ns
divisions or unguarded narrowing casts remain).

### Findings
- [x] (suggestion, Copilot suppressed) `isoUtc()` truncating ns->ms
  division shifts the displayed second within 1 ms past a pre-epoch
  second boundary; floor-divided — `src/time_bar_widget.cpp:57`
- [x] (suggestion, Copilot suppressed) `indexedTiles()` negative guard
  still let over-uint32 row/col wrap to small values; cast now bounded
  both sides + fixture rows — `src/survey_index_bridge.cpp:90`
- [x] (suggestion, local sweep) animation interpolation subtracted
  `anim_to_ns_ - anim_from_ns_` in int64, overflowing when one end is at
  a saturation extreme; computed in double via new `saturateNs()` —
  `src/time_bar_widget.cpp:94`

### False positives
- (none)
