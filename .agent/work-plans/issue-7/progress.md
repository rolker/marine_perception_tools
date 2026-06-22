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
