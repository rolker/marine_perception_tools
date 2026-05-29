---
issue: 1
---

# Issue #1 — sea_surface_tuner: interactive C++/Qt tool to replay bags and tune segmentation→costmap params

## Plan Authored
**Status**: complete
**When**: 2026-05-29 10:46 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-1/plan.md` at `4a87076`
**PR**: https://github.com/rolker/marine_perception_tools/pull/2 (`[PLAN]` prefix)
**Phases**: single (splittable into headless-core + Qt-UI PRs if review prefers)

### Open questions
- [ ] Confirm a usable #186 bag path (oak_forward segmentation + camera_info + TF) for manual verify — does not block implementation.
- [ ] Single PR vs stacked (headless-core PR-A + Qt-UI PR-B) — recommend single; surface for decision.
