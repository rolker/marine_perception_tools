---
issue: 37
---

# Issue #37 — Write docs/survey_explorer.md — the explorer's design document

## Integrated Review
**Status**: complete
**When**: 2026-09-03 12:42 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))

**PR**: #38 at `de61f68`
**Sources**: 2 (Copilot R1 @ `de61f68`, CI rollup). No prior local timeline — this doc PR was written directly, without a plan phase, so there are no cross-source confirmations to report.
**Cross-source confirmations**: 0
**CI**: all-pass (`build-and-test` success; docs-only PR, so green carries no information about the content)

### Findings
- [ ] (medium, Copilot R1) The "Not a general GIS" bullet **enumerates** `uma-ADR-0010` D12's constraints (WGS84/ellipsoidal only, no arbitrary CRS, no styling, no replacing GDAL/QGIS) instead of pointing at them — and the **immediately following bullet** states the rule it breaks: "This document references them; it must never restate them, or it becomes a fourth namespace disagreeing about the same data." The two bullets are adjacent. The restatement is also already lossy: D12 lists "no external spatial database" and the doc drops it, which is the divergence mechanism demonstrating itself on the first draft. Fix: replace the enumeration with a pointer to D12, keeping only the claim that the explorer inherits that scope. — `docs/survey_explorer.md:126-128`

### False positives
- (Copilot R1) "un-averaged" should be "unaveraged" (`docs/survey_explorer.md:22`). Declined for consistency, not because the suggestion is wrong in isolation: **"un-averaged" is the established phrasing for this concept across the project** — it originates in uma#258's own body ("target work needs the **raw, un-averaged data behind a location**") and `unh_marine_autonomy/docs/sonar_ecosystem.md:264` repeats it verbatim. Changing one of the three documents that describe the same concept would create the divergence, not remove it. If the spelling is to change it should change in all three, which is a separate (and low-value) edit.

### Process note
An earlier command in this triage ran `grep` from the wrong worktree (the `unh_marine_autonomy` issue-367 tree rather than this one), which is why `docs/sonar_ecosystem.md` appeared in a search scoped to this repo. Re-run in the correct tree before concluding; the hyphenation finding above rests on the re-run, not the first result.
