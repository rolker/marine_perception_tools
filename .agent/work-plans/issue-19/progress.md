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
