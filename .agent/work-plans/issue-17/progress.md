---
issue: 17
---

# Issue #17 — Interval loader: open sidescan_target_viewer cued to a survey-index pass

## Issue Review
**Status**: complete
**When**: 2026-07-14 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #17
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Only what's needed | OK | Issue explicitly lists non-goals and calls for a "deliberately thin" CLI flag surface. Interval selection logic is on the critical path for stages 2–3 of the umbrella. |
| Improve incrementally | OK | Bounded single-PR scope; non-goals defer to later stages (#258). |
| Test what breaks | OK | Acceptance criteria require tests for interval-selection logic, bag-I/O-free where possible — this matches the existing test approach in the repo. |
| A change includes its consequences | Watch | The implementation will introduce CLI arguments and possibly a new seek path in `buildIndex`. The `.agents/README.md` package inventory and the CLI usage note in `main()` will need updating in the same PR. |
| Capture decisions, not just implementations | Watch | The issue surfaces a key architectural choice: whether to seek-based index only the time window (requiring TF context from bag start) or to do whole-bag index then reposition. The issue wisely says "if unavoidable, measure and document it" — the PR should capture that rationale in a code comment or PR description, not just leave it implicit. |
| Human control and transparency | OK | A thin `--start`/`--end` CLI is transparent and scriptable. No hidden automation. |
| Workspace vs. project separation | OK | All changes are in `marine_perception_tools` (project repo). No workspace contamination risk. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| ADR-0008 — Follow ROS 2 Conventions | Yes | C++ source changes; existing code already follows conventions (Apache 2.0 headers, ROS 2 naming). New code must match. |
| ADR-0001 — Adopt ADRs | Watch | If the seek-vs-whole-bag decision is a durable architectural choice, it warrants a comment or ADR. The issue already asks for it to be documented — satisfy with a code comment or PR description rather than a new ADR unless the choice spans packages. |
| ADR-0002 — Worktree isolation | OK | Worktree already exists (`feature/issue-17`). |

### Consequences

- **`.agents/README.md` package inventory**: Adding `--start`/`--end` CLI arguments to `sidescan_target_viewer` changes the documented CLI surface; the entry under `main.cpp` should be updated in the same PR.
- **`SidescanBagOptions`**: If a time-window constraint is added as an option, downstream callers (none currently outside the viewer) need to be aware. Likely scoped to `main()` argument parsing for now — fine at this stage.
- **Seek path documentation**: The acceptance criterion "does not read the whole bag (or, if unavoidable, measured and documented)" must be satisfied in the PR. The pitfall note in `.agents/README.md` already flags that the BufferPlan Extend optimization is unwired — the interval loader should not regress that or silently assume seek will work on all rosbag2 storage plugins.

### Recommendations

- The `sidescan_viewer_main.cpp` `main()` currently uses plain `argc`/`argv` with no argument-parsing library. Qt provides `QCommandLineParser` — use it for `--start`/`--end` parsing since Qt is already linked and the app is Qt-based. This avoids adding a new dependency.
- The interval-selection logic (converting `[t_start, t_end]` to `[dist_lo, dist_hi]` against the ping index) should live in a Qt-free, bag-I/O-free function so it can be unit-tested without a display server. The existing `sidescan_core` library is the natural home.
- Document in the PR whether the index build still reads the whole bag (for TF continuity) or only the windowed slice. Either is acceptable; the acceptance criterion just requires the choice be explicit.

### Actions
- [ ] Update `.agents/README.md` CLI surface note when the `--start`/`--end` arguments are added.
- [ ] Document (code comment or PR description) the seek-vs-whole-bag architectural choice and whether TF requires reading from bag start.
- [ ] Ensure interval-selection logic is in a Qt-free, testable function (not embedded in the Qt window callback).

## Plan Authored
**Status**: complete
**When**: 2026-07-14 00:00 +00:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-17/plan.md` at `68fbbb2`
**Branch**: feature/issue-17 at `68fbbb2`
**Phases**: single

### Open questions
- [ ] No open questions — plan is review-plan-ready.

## Plan Review
**Status**: complete
**When**: 2026-07-14 15:26 +00:00
**By**: Claude Code Agent (Claude Opus) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-17/plan.md` at `68fbbb2`
**PR**: PR-less (`--issue 17`; reviewed from `feature/issue-17` worktree)
**Verdict**: changes-requested

### Findings
- [ ] (must-fix) Cue target conflicts with trailing-window semantics: `distance_window(head,…)` paints `[head − window_len, head]` (`distance_buffer_policy.hpp:40`), so cueing the scrub head to `dist_lo` shows the window BEFORE the pass, not the pass itself — cue to `dist_hi` (or `dist_lo + window_len_m_`, clamped to total) — `plan.md:42`
- [ ] (suggestion) ISO-8601 parse: `QDateTime::fromString(s, Qt::ISODateWithMs)` defaults to LocalTime when the string has no offset, mis-converting the epoch; force/verify UTC and reject an invalid `QDateTime` with a clear error — `plan.md:45`
- [ ] (suggestion) `.agents/README.md` inventory/layout describe only `sea_surface_tuner`/`main.cpp`; the sidescan viewer is absent, so "add to the package inventory row" needs a viewer entry, not an append to the tuner row — `plan.md:51`
- [ ] (suggestion) Flag naming: `sea_surface_tuner` already uses `--start-s`/`--end-s` (seconds); keep the viewer's suffix-free `--start`/`--end` and make the ns-or-ISO meaning explicit in `--help` — `plan.md:45`
- [ ] (suggestion) Apply the cue at the END of the `done` block in `onIndexProgress()` (after the range is finalized at `sidescan_viewer_window.cpp:872`) and clear `pending_cue_*` there, so it isn't clobbered — `plan.md:43`
