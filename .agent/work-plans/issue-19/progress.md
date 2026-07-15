---
issue: 19
---

# Issue #19 — Survey overview pane for sidescan_target_viewer

## Issue Review
**Status**: complete
**When**: 2026-07-14 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #19
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Actions
- [ ] Resolve and record the coordinate frame decision (GGGS lat/lon with scale factor vs local ENU) in the plan — the `SidescanCanvas` ENU frame vs a new lat/lon widget is a non-trivial design fork with correctness and reuse consequences.
- [ ] Capture the tile load strategy decision (eager whole-store vs visible-extent) in the plan with rationale — relevant for scalability to larger surveys (Isles of Shoals noted in issue).
- [ ] Add `marine_tiled_raster_store` and `marine_survey_index` to `marine_perception_tools/package.xml` and CMakeLists.txt — currently absent; omitting these breaks rosdep and ament dependency resolution.
- [ ] Plan must include a unit test for pass-query logic using an in-memory SQLite DB (issue specifies this; plan should name the test file and describe the fixture).
- [ ] Clarify "tile placement verified against known store tiles" in the acceptance criteria — decide whether this is a unit test, an integration test, or a manual check, and scope it in the plan accordingly.

## Plan Authored
**Status**: complete
**When**: 2026-07-14 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-19/plan.md` at `16f7797`
**Branch**: feature/issue-19 at `16f7797`
**Phases**: single

### Open questions
- [ ] Tile load path: confirm `--stores <dir>` subdirectory naming (e.g. `bathymetry/`) for bathy GeoTIFFs
- [ ] Colormap for depth basemap: which `marine_colormap` palette to use by default

## Plan Review
**Status**: complete
**When**: 2026-07-14 17:32 +00:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-19/plan.md` at `16f7797`
**PR**: PR-less (`--issue` mode)
**Verdict**: changes-requested

Architecturally sound: correct decomposition (headless testable bridge / Qt
canvas / window / CLI), accurate file targeting, and referenced dep APIs
(`tilesForBoundingBox`, `queryPasses`, `loadTiles`, `PassRow`) verified present
in `core_ws`. Two review-issue findings did not make it into the plan; both are
small inline amendments, not structural rework.

### Findings
- [ ] (must-fix) `package.xml` not updated — plan adds `marine_survey_index`, `marine_tiled_raster_store`, SQLite3 to CMakeLists only; needs `<depend>marine_survey_index</depend>`, `<depend>marine_tiled_raster_store</depend>`, `<depend>libsqlite3-dev</depend>` or rosdep/ament resolution breaks (review-issue action #3) — `plan.md:52` (Files-to-Change) & `plan.md:84` (Consequences)
- [ ] (suggestion) Tile-placement verification unscoped (review-issue action #5) — the `cos(lat)` lat/lon→pixel projection is correctness-critical but untested; extract it as a pure unit-testable function or document a manual acceptance check — `plan.md:46`, `plan.md:70`
- [ ] (suggestion) Note SQLite3 `target_link_libraries` linking (not just `find_package`) in the CMake change — `plan.md:44`

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-14 19:09 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-19 at `8d9ae62`
**Mode**: pre-push
**Depth**: Deep (reason: 1242 changed lines / 15 files; correctness-critical geo projection)
**Must-fix**: 2 | **Suggestions**: 4
**Round**: 1 | **Ship**: continue — two genuine must-fixes; the tile-load exception gap is a real robustness/contract defect worth fixing before push

Specialists: static analysis (cpplint clean; cppcheck syntaxError on TEST macros = gtest false positive), governance (all consequences Done — CLI doc lives in `.agents/README.md` per the #17 precedent, updated), plan drift (none — projection extraction, package.xml deps, SQLite linking all implemented; both plan open questions resolved in code), 2 fresh-context Claude adversarial passes (Lens A + Lens B). Copilot off (default). All cross-package API calls verified against the actual dependency headers in `core_ws`.

### Findings
- [x] (must-fix) Ctor `loadStoreTiles` calls `tileRasterCount()`/`loadTiles()` unguarded — both `@throws`; one bad tile kills the whole overview window (breaks documented graceful-degradation) and misreports as an index error — `src/survey_overview_window.cpp:141,151-152`
- [x] (must-fix) Initial view fit uses pre-layout widget size and is never recomputed (`have_fit_` set true at ctor-time; no `resizeEvent`/`showEvent`) — basemap opens mis-scaled — `src/survey_overview_canvas.cpp:54-71,83-85`
- [x] (suggestion) `survey_index_bridge` directly includes `marine_autonomy/gggs.h` but only depends on it transitively via `marine_survey_index`; add `marine_autonomy` to CMake + package.xml for directness — `CMakeLists.txt:122-125`, `package.xml`
- [x] (suggestion) Multi-tile click lists one physical pass as several per-tile segments; consider coalescing per `(bag,sensor,topic)` — `src/survey_overview_window.cpp:210-229`
- [x] (suggestion) Mixed-level / non-band-0 stores silently partial-render; guard or warn — `src/survey_overview_window.cpp:129-152`
- [x] (suggestion) `onHoverGeo` `statusBar()->showMessage()` transiently hides the non-permanent `status_` label on every mouse-move; use `addPermanentWidget` or a separate label — `src/survey_overview_window.cpp:108-110,250-254`

## Implementation
**Status**: complete
**When**: 2026-07-14 19:37 +00:00
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-19 at `3a37c3e`
**Addressed**: Local Review (Pre-Push) — 2026-07-14 19:09 +00:00 at `8d9ae62` (all 6 unchecked findings; 2 must-fix, 4 suggestions)
**Commits**: `92c9c64`, `e531459`, `107be81`, `8b4e47a`, `65127fa`, `ecae9dd`, `3a37c3e`

### Actions
- [x] (must-fix) Guard throwing tile loads — wrapped `tileRasterCount()` and `loadTiles()` in try/catch that degrades to an empty map with a status message (pass query still works), so one bad tile no longer aborts window construction or misreports as an index error — `src/survey_overview_window.cpp` (`92c9c64`)
- [x] (must-fix) Basemap opens correctly scaled — `setTiles` no longer fits at ctor-time (pre-layout size); the fit is deferred to the first paint and a new `resizeEvent` refits at the real laid-out size until the user zooms/pans (`user_adjusted_` guard) — `src/survey_overview_canvas.{cpp,hpp}` (`e531459`)
- [x] (suggestion) Direct `marine_autonomy` dependency — added `find_package(marine_autonomy)` + `ament_target_dependencies` for both `survey_index_bridge` and `sidescan_target_viewer` (both include `marine_autonomy/gggs.h` directly) and `<depend>marine_autonomy</depend>` — `CMakeLists.txt`, `package.xml` (`107be81`)
- [x] (suggestion) Coalesce per-tile pass segments — new `coalescePasses()` merges segments sharing `(bag,sensor,topic)` whose time windows overlap or sit within a 5 s gap (one transit across adjacent tiles) into one list row, summing ping counts; distinct revisits stay separate — `src/survey_overview_window.cpp` (`8b4e47a`, formatting `3a37c3e`)
- [x] (suggestion) Warn on mixed-level stores — scan collects all tile levels, renders the lowest deterministically, and appends a "mixed store: ignoring levels …" note to the status instead of silently dropping other levels — `src/survey_overview_window.cpp` (`65127fa`)
- [x] (suggestion) Hover read-out no longer hides the status label — added a permanent `hover_` label (`addPermanentWidget`); `onHoverGeo` writes coords there instead of `statusBar()->showMessage()` — `src/survey_overview_window.{cpp,hpp}` (`ecae9dd`)

### Checks
- `ament_cpplint` on all four changed source/header files: **No problems found**.
- `ament_uncrustify` on all four: clean after the `3a37c3e` continuation-indent fix.
- Pre-commit hooks (cmake-lint, check-xml, whitespace/EOL, commit-identity) passed on every commit (no `--no-verify`).
- Full `colcon build`/gtest **not run**: the `core_ws` underlay is not built in this worktree (empty `core_ws/install`), so `find_package(marine_autonomy …)` can't resolve here. Compile/`ament_lint_auto` test coverage is deferred to the re-review / CI, which builds the underlay. Changes were reasoned against the dependency headers in `layers/main/core_ws/src` (PassRow fields, `tileRasterCount`/`loadTiles` throw contracts).

### Next step
Lifecycle: **Implementation** → **review-code** (re-review the fixes). Hand off to a fresh-context sub-agent:

    .agent/scripts/dispatch_subagent.sh --mode in-process --issue 19 --skill review-code

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-14 19:57 +00:00
**By**: Claude Code Agent (Claude Opus)
**Verdict**: changes-requested

**Branch**: feature/issue-19 at `485e9cf`
**Mode**: pre-push
**Depth**: Deep (reason: >1000 changed lines / 15 files; correctness-critical cos(lat) geo projection; cross-package)
**Must-fix**: 1 | **Suggestions**: 0
**Round**: 2 | **Ship**: recommended — one contained, precise must-fix (down from 2 in round 1, not rising); address and ship rather than another full round

Round-2 re-review. Verified all 6 round-1 findings (2 must-fix, 4 suggestions) are actually
fixed in the diff (throwing tile loads guarded; fit deferred to laid-out size + resizeEvent
refit; direct marine_autonomy dep; coalescePasses; permanent hover label). Static analysis
clean (ament_cpplint + ament_uncrustify: no problems on all changed sources). Two fresh-context
Claude adversarial passes: Lens B (systemic/safety) found nothing — SQLite handle lifecycle,
unique_ptr/WA_DeleteOnClose window ownership, exception guarding, CMake/package.xml deps all
sound. Lens A (logic) found one real defect, verified against the actual loadTiles/loadTile
implementation in core_ws. Copilot off (default). Not built/tested here (core_ws underlay
unbuilt in this worktree); compile + gtest deferred to CI.

### Findings
- [x] (must-fix) Mixed-level-store handling is dead code / contradicts its claim: `loadTiles` loads every `.tif` at the chosen level and throws on any off-level tile, so a mixed store hits the catch (empty map + "failed to load") and never renders "lowest + warn" — the `level_warning` branch and `%5` placeholder are unreachable. Fix to match contract or drop the dead code + false comment/log claim — `src/survey_overview_window.cpp:238-247,294`

## Implementation
**Status**: complete
**When**: 2026-07-14 20:20 +00:00
**By**: Claude Code Agent (Claude Fable 5)

Addressed the round-2 must-fix (host-inline):

- [x] Store tiles now load per-tile (`tileRasterCount` + `loadTile`, per-tile
  try/catch) instead of `loadTiles()`, which throws on any off-level tile and
  made the mixed-level branch unreachable. Mixed stores genuinely render the
  lowest level with the warning, a corrupt tile is skipped (and counted in the
  status) instead of emptying the map — `src/survey_overview_window.cpp` (`ea2e8c3`)

Verification: rebuild + full suite green — **272 tests, 0 failures, 41
skipped**; real-data smoke (Massabesic index + survey store) alive at timeout.

## Published
**Status**: complete
**When**: 2026-07-14
**By**: Claude Code Agent (Claude Fable 5)

Publish checkpoint approved by Roland. Branch `feature/issue-19` pushed;
PR opened: https://github.com/rolker/marine_perception_tools/pull/20
(base `jazzy`, `Closes #19`).

## Integrated Review
**Status**: complete
**When**: 2026-07-15 08:48 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #20 at `ccc37d7` (local head `b447700` adds the CI fix, unpushed)
**Sources**: 4 (Copilot R1 @ `53f511f`, Copilot R2 @ `ccc37d7`, Local Review timeline, CI rollup)
**Cross-source confirmations**: 0
**CI**: failures-noted — `build-and-test` FAILED: `find_package(marine_autonomy)` unresolved; the mpt CI workflow only kept marine_interfaces + marine_contacts from its unh_marine_autonomy clone. Fixed locally in `b447700` (stages the full 5-package chain: marine_autonomy, marine_tiled_raster_store, marine_backscatter, marine_sidescan_mosaic, marine_survey_index).

### Findings
- [x] (must-fix, Copilot R2 ×6 grouped) No defined geo frame when store tiles are absent or before the first fit: `GeoView` defaults to (0°, 0°) @ 1 px/deg, yet `wheelEvent`/`mouseMoveEvent`/`mouseReleaseEvent` project through it unguarded (bogus `hoverGeo`/`clicked` near Null Island, and wheel/pan set `user_adjusted_`), while the window status text and ctor docstring claim "pass queries still work" with an empty map — the click can't be aimed, so the claim is false as shipped — `src/survey_overview_canvas.cpp:120-172`, `src/survey_overview_window.cpp:209-213,257-262`, `src/survey_overview_window.hpp:41-44`. Recommended fix: gate geo-emitting interactions on `have_fit_`, and make the claim TRUE by fitting the view from the survey index extent when no store tiles load (extent query on SurveyIndexBridge → canvas fallback fit bounds), keeping the "No store tiles loaded" backdrop.
- [x] (minor, Copilot R1) Test fixture `exec()` streams sqlite3's `err` into the assertion without `sqlite3_free()` — leaks only on the failure path, still trivially fixable — `test/test_survey_index_bridge.cpp:60-65`.
- [x] (must-fix, CI rollup) `build-and-test` red: missing uma sibling packages in the CI workspace — fixed in `b447700` (`.github/workflows/ci.yml`).

### False positives
- none — all six R2 comments share the one valid root cause above; R1's leak is real (if minor).

## Implementation
**Status**: complete
**When**: 2026-07-15 10:35 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**Branch**: feature/issue-19 at `2780582`

Addressed all open Integrated Review findings (round 1):
- `b447700` ci: stage the full uma dependency chain (marine_autonomy,
  marine_tiled_raster_store, marine_backscatter, marine_sidescan_mosaic,
  marine_survey_index) — fixes the red `build-and-test` check.
- `4a56bea` fix(overview): SurveyIndexBridge::extent() + canvas fallback
  fit + have_fit_ event gating — resolves all six Copilot R2 comments at
  their shared root cause; the "click-to-query without store tiles"
  promise is now true (fitted to the index extent) instead of reworded.
- `2780582` test(overview): sqlite3_free in the bridge fixture (Copilot
  R1) + extent() tests (bounds pinned against gggs::GridIndex accessors;
  empty index → nullopt).

Verified: worktree build clean; 274 tests, 0 failures (was 272, +2
extent tests); offscreen smokes with the real Massabesic index — normal
stores path and missing-stores fallback path both survive startup.

## Integrated Review
**Status**: complete
**When**: 2026-07-15 11:43 -04:00
**By**: Claude Code Agent (Claude Fable 5)

**PR**: #20 at `936dddb` (Copilot R3 @ `c45ed7d`: 3 comments; R4 @ `936dddb`: no new comments)
**Sources**: 2 (Copilot R3, CI rollup)
**Cross-source confirmations**: 0
**CI**: all-pass — `build-and-test` GREEN at `936dddb` after the two workflow fixes (uma package chain `b447700`, geographic_info ros2-branch clone `936dddb` — released jazzy geodesy predates geodesy/geodesics.h).

### Findings
- [x] (minor, Copilot R3) Filename-parsed store level unvalidated; real failure mode is worse than flagged: by_level renders its LOWEST key, so a junk "-1_x_y.tif" wins level selection and blanks the real tiles — fixed `b768b9d` (range check vs gggs::levels at scan time) — `src/survey_overview_window.cpp`
- [x] (trivial, Copilot R3 ×2) `::testing::TempDir()` concatenated without explicit path separator — fixed `b768b9d` — `test/test_survey_index_bridge.cpp`

### False positives
- none.
