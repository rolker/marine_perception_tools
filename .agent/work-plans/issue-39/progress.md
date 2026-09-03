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

## Plan Review
**Status**: complete
**When**: 2026-09-03 13:42 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-39/plan.md` at `72116fb`
**PR**: PR-less
**Verdict**: changes-requested

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Matches the issue's own suggested steps 1-2; step 3 correctly deferred. |
| Issue alignment | Good | All 5 review-issue action items (beamwidth convention test, PingGeometry plumbing, not-reported fallback, gate-sync, README update) are explicitly addressed. |
| File targeting | Good | Verified against source: single call site of `extend_surface_for_drape()` (`sidescan_viewer_window.cpp:1545`), no other `PingGeometry` construction sites outside tests, `.agents/README.md` has no existing drape-specific text to cross-reference (see Documentation finding). |
| Consequences | Good | Consequences table verified accurate: `terrain.uncertainty[]`/`intensity[]` are read only as an `isfinite()` "measured" boolean flag on the drape path (`sidescan_viewer_window.cpp:1186,1359`), never as a numeric value — confirms the plan's step-5 claim these fields are display-only there and safe to interpolate under the same "NaN unless all 4 finite" rule as depth. |
| Documentation & instruction impact | Needs work | Section is present and non-silent, but factually inaccurate — see Finding 3. |
| Principle alignment | Good | Test-driven, minimal, both blur sources addressed together per the Quality Standard. |
| ADR compliance | N/A | Correctly noted — no ADR governs sonar geometry/resampling. |
| ROS conventions | N/A | No topics/params/messages changed; internal algorithm + one Qt spinbox. |

### Findings

1. **(must-fix) [Approach, step 3]** — The along-track "weighted accumulation" as described is likely a no-op for the painted value, and its interaction with the existing gap-filling role of `half_width_m` is unresolved — `plan.md:84-109`.

   Within one call to `drapePing()`, a single march step `t` produces exactly **one** amplitude value (the step-4 box average), which today is stamped identically across every offset in `offsets` (built from `half_width_m`, the ping-spacing-derived strip width — `sidescan_drape.cpp:114-119`). Step 3 proposes weighting each offset by a triangular kernel over `0.5*tx_beamwidth_rad*slant` and resolving `weighted_amplitude = Σ(weight·amplitude)/Σ(weight)` per touched cell. Since `amplitude` is the *same* constant for every offset at a given `t` (there is no per-offset amplitude data within a single ping — sidescan has no along-track samples finer than one ping), this ratio algebraically collapses to that same constant regardless of the weights: a weighted average of a repeated constant is the constant. So, on the most direct reading, the described accumulation cannot change what gets painted.

   Two ways to make this a real effect, and the plan doesn't pick one:
   - **Narrow the paint extent** to the kernel half-width (drop offsets beyond it, rather than merely down-weighting them within the still-full `half_width_m` strip). This would meaningfully tighten the along-track footprint at far range, but at near range (<9 m, per the issue's own numbers) the beam footprint (3.8-7.7 cm) is *narrower* than the ping-to-ping spacing (6.9 cm) that `half_width_m`'s comment (`sidescan_drape.cpp:73-75`) says exists specifically "so consecutive pings tile the grid without grey gaps between their rays." Narrowing to the footprint reopens exactly the gap problem that code comment says the current width was chosen to avoid — the plan doesn't address this trade-off.
   - **Blend across neighbouring pings** at cells where footprints overlap (the physically correct source of along-track anti-aliasing, since resolution finer than one ping only exists *between* pings, not within one). But step 3's own bullet 3 says the resolved candidate feeds "the **existing unchanged** conflict-resolution path" (`score` winner-take-all), which explicitly rules this out.

   The plan needs to say concretely which of these (or a third option) is intended, and — if it's the first — reconcile it with the near-range gap-filling requirement `half_width_m` currently satisfies. As written, an implementer following the prose literally could ship code that computes and normalizes weights for no visible effect on drape sharpness, while believing the along-track half of the issue's blur has been addressed.

2. **(suggestion) [Test plan, across-track anti-aliasing case]** — `plan.md:200-204`. A box average over the raw samples a march step spans is the textbook anti-aliasing operation for downsampling, but it will legitimately *dilute* a bright sub-window target's peak amplitude in proportion to how much of the averaging window it occupies (e.g. a single strong sample among ~3 averaged samples nets roughly a third of its raw value) — that dilution is correct behaviour, not a bug. The stated acceptance criterion — "preserves the target's amplitude and position" — reads as literal equality, which this kernel will not produce. Recommend the plan specify a quantitative, dilution-aware assertion (e.g. amplitude scales with the target's fractional coverage of the window, and never collapses to exactly the pre-fix nearest-sample outcome of "sometimes present, sometimes 0") rather than "preserved," to avoid the test either being flaky/impossible to satisfy as literally written or silently asserting something weaker than intended.

3. **(suggestion) [Documentation & Instruction Impact]** — `plan.md:248-252`. The plan states `.agents/README.md`'s inventory row "currently state[s] 'Nearest sample per cell, no averaging'" — verified against source, this phrase and any drape-kernel description do not currently appear anywhere in `.agents/README.md` (grep confirms no hits); only `sidescan_drape.hpp`'s header comment (line ~25) has it. The correct framing is that README.md needs a **new** drape-kernel description added (it currently has none), not a correction of an existing stale claim. Minor — doesn't change the Files to Change list, which already lists both files correctly — but worth fixing the prose since the plan's own Principles Self-Check claims doc quantities were "verified against source."

### Summary

The plan is well-researched, correctly scoped against the issue and its review, and the grid-decoupling (step 5) and not-reported-fallback (step 2) mechanics check out cleanly against the source (verified the single call site, the `terrain.uncertainty` display-only usage, the membrane-fill's NaN-triggers-fill behaviour, and the `PingInfo.msg` "may be empty if not reported" comment). However, the along-track kernel (step 3) — the half of the plan addressing the issue's *dominant* blur source (16:1 aspect ratio at swath edge) — is under-specified to the point of being mathematically inert as literally described, and needs to be made concrete before implementation.

### Recommended Actions

- [ ] Resolve Finding 1 — specify precisely what dimension the along-track weighted accumulation operates over, and reconcile the chosen mechanism with `half_width_m`'s near-range gap-filling role, before implementation starts.
- [ ] Tighten the across-track anti-aliasing test's acceptance criterion (Finding 2) to a dilution-aware assertion.
- [ ] Correct the Documentation & Instruction Impact wording (Finding 3) — README.md needs a new section, not a correction of existing text.
- [ ] Open question 1 (triangular vs. Gaussian) is downstream of Finding 1 — resolve the mechanism first; the kernel-shape choice may need to be re-derived once the accumulation dimension is fixed.
- [ ] Open question 2 (drape-cell default) — no objection to either option; operator call.
