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
- [ ] (must-fix) Ctor `loadStoreTiles` calls `tileRasterCount()`/`loadTiles()` unguarded — both `@throws`; one bad tile kills the whole overview window (breaks documented graceful-degradation) and misreports as an index error — `src/survey_overview_window.cpp:141,151-152`
- [ ] (must-fix) Initial view fit uses pre-layout widget size and is never recomputed (`have_fit_` set true at ctor-time; no `resizeEvent`/`showEvent`) — basemap opens mis-scaled — `src/survey_overview_canvas.cpp:54-71,83-85`
- [ ] (suggestion) `survey_index_bridge` directly includes `marine_autonomy/gggs.h` but only depends on it transitively via `marine_survey_index`; add `marine_autonomy` to CMake + package.xml for directness — `CMakeLists.txt:122-125`, `package.xml`
- [ ] (suggestion) Multi-tile click lists one physical pass as several per-tile segments; consider coalescing per `(bag,sensor,topic)` — `src/survey_overview_window.cpp:210-229`
- [ ] (suggestion) Mixed-level / non-band-0 stores silently partial-render; guard or warn — `src/survey_overview_window.cpp:129-152`
- [ ] (suggestion) `onHoverGeo` `statusBar()->showMessage()` transiently hides the non-permanent `status_` label on every mouse-move; use `addPermanentWidget` or a separate label — `src/survey_overview_window.cpp:108-110,250-254`
