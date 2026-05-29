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
**Phases**: stacked — Milestone A (pipeline + viewer + #22-stable dock, now) / Milestone B (#22 knob set, after unh_marine_perception#22-P1)

### Open questions
- [ ] Confirm a usable #186 bag path (oak_forward segmentation + camera_info + TF) for manual verify — does not block implementation.
- [x] PR split — resolved (Roland, 2026-05-29): stacked A/B, driven by the #22 param-model coupling (not single PR). Plan revised accordingly.
- [ ] Milestone B blocks on `unh_marine_perception#22` Phase 1 (plan PR #25) landing the new param fields on `jazzy`.

## Plan Revised
**Status**: complete
**When**: 2026-05-29 11:18 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-1/plan.md` (revised)
**Why**: discovered `unh_marine_perception#22` (plan PR #25) replaces the tuner's parameter model. Re-scoped to stacked Milestone A (algorithm-agnostic pipeline + viewer + #22-stable knobs, buildable now) / Milestone B (#22 knob set, after #22-P1). Param dock built table-driven so B is a data edit. See plan `## Implementation Notes`.
