---
issue: 39
---

# Issue #39 — Sidescan drape discards ~5 of every 6 across-track samples — nearest-sample resampling, not a resolution limit

## Issue Review
**Status**: complete
**When**: 2026-09-03 13:04 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Issue**: #39
**Comment**: (best-effort post follows this entry; not recorded inline)
**Scope verdict**: well-scoped

### Scope Assessment

**Well-scoped?** Yes. Steps 1-2 (footprint-weighted accumulation in
`drapePing()`; decouple the drape grid cell from the CUBE surface cell) are
explicitly scoped as self-contained and testable against a synthetic target;
step 3 (on-demand rasterization at view resolution) is explicitly deferred
to a later decision, not bundled into this issue. Right size for one PR.

**Right repo?** Yes — `drapePing()` lives in
`src/sidescan_drape.cpp` in this repo (verified by reading the source: the
"nearest sample, no averaging" comment at line 184 matches the issue's claim
exactly, and the along-track strip-paint logic at lines 110-119/195-208
matches the described loss mechanism).

**Dependencies**: "Part of #36" (open umbrella, "[Umbrella] Survey explorer
direction"). No other open issue blocks #39 itself.

### Principle Alignment

| Principle | Status | Notes |
|---|---|---|
| Test what breaks | OK | Issue proposes a synthetic-target test (narrower-than-cell feature must survive at correct amplitude/position) — targets the actual regression, not framework glue. |
| A change includes its consequences | Watch | `PingGeometry` (src/sidescan_geometry.hpp:91-100) currently has **no beamwidth field at all** — the suggested kernel input (`tx_beamwidth x range`) isn't plumbed yet. Implementation needs to add it and thread it from `PingInfo.tx_beamwidths` through `sidescan_bag_session.hpp`'s ping construction; the issue's "Suggested scope" doesn't call this out as its own step. |
| Capture decisions | Watch | `marine_acoustic_msgs/PingInfo.tx_beamwidths`/`rx_beamwidths` are the **full -3 dB width in radians**, not a half-extent — confirmed via `/opt/ros/jazzy` message headers and via `rolker/rviz_sonar_image#8` (open), which documents this exact convention biting a *different* consumer (fan renders 2x too wide) and cross-references a *third* consumer bug (`cube_bathymetry#30`, full width read as degrees). This is a recurring "consumer disagrees with the .msg convention" class. The issue itself flags the question ("the same convention question applies to any footprint kernel built from it") but doesn't resolve it — plan-task/implementation should explicitly decide and test full-vs-half before building the kernel, not rediscover the bug a third time. |
| Human control and transparency | OK | Internal rendering-fidelity fix; no new topics/params/behavior surprises beyond the drape looking sharper. |
| Improve incrementally | OK | Larger option (on-demand rasterization) explicitly deferred rather than bundled. |

### ADR Applicability

| ADR | Triggered | Notes |
|---|---|---|
| 0008 — ROS 2 conventions | No | No new package, message, launch file, or param — internal C++ function change only. |
| 0013 — progress.md vocabulary | Yes (process) | This review itself; downstream plan-task/implementation entries follow the standard sequence. |

### Consequences

- `PingGeometry` needs a beamwidth field (or equivalent) added and populated
  from `PingInfo.tx_beamwidths` in `src/sidescan_bag_session.hpp` — not
  mentioned in the issue's scope list but required by step 1.
- Many producers leave `tx_beamwidths`/`rx_beamwidths` empty (per
  `rviz_sonar_image#8`'s "why it's latent today" note) — the kernel needs an
  explicit, documented fallback for the empty case, consistent with
  `drapePing()`'s existing graceful-degradation pattern for missing geometry
  (`++out.pings_skipped`).
- `extend_surface_for_drape()` (sidescan_drape.cpp:232) has a comment
  explicitly noting it must "mirror drapePing()'s ping-level gate" — any
  gating-condition change in step 1 needs the mirrored comment/logic kept in
  sync in the same PR.
- `.agents/README.md` Common Pitfalls / Package Inventory should gain an
  entry once implemented (new kernel parameter(s), drape-grid resolution now
  independent of the CUBE cell) — per the doc's own verified-parameter
  convention.

### Recommendations

- Resolve and test the `tx_beamwidths` full-width-vs-half-extent convention
  explicitly before wiring it into the kernel — two sibling repos already
  got this wrong once each (`rviz_sonar_image#8`, `cube_bathymetry#30`).
- Add the beamwidth-plumbing step (PingGeometry field + population from
  PingInfo) as an explicit item in the plan, since it's a prerequisite the
  issue's scope list doesn't list.
- Decide and document the empty-`tx_beamwidths` fallback behavior.

### Actions
- [ ] Resolve and test the tx_beamwidths full-vs-half-extent convention explicitly before using it in the kernel (sibling bug pattern hit twice already: rviz_sonar_image#8, cube_bathymetry#30).
- [ ] Add a beamwidth field to PingGeometry and plumb it from PingInfo.tx_beamwidths in sidescan_bag_session.hpp — not currently present, required by suggested step 1 but not listed in the issue's scope.
- [ ] Decide and document the fallback when tx_beamwidths is empty (common per rviz_sonar_image#8).
- [ ] Keep extend_surface_for_drape()'s mirrored ping-level gate in sync with any gating change made in drapePing().
- [ ] Update .agents/README.md (Common Pitfalls / Package Inventory) once the kernel + decoupled drape-grid resolution land.


## Plan Authored
**Status**: complete
**When**: 2026-09-03 13:34 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-39/plan.md` at `72116fb`
**Branch**: feature/issue-39 at `72116fb`
**Phases**: single

### Open questions
- [ ] Triangular vs. Gaussian along-track kernel shape — plan defaults to triangular for simplicity/cost; confirm before implementation since it changes the half-width test assertion.
- [ ] Default value for the new drape-cell-size UI control — plan suggests ~0.02 m (near measured GCV across-track sample spacing); confirm a fixed default is acceptable vs. defaulting to the CUBE cell size (opt-in decoupling).
