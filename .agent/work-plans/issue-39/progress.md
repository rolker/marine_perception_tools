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

## Plan Authored
**Status**: complete

**Plan**: `.agent/work-plans/issue-39/plan.md` at `a388281`
**Branch**: feature/issue-39 at `a388281`
**Phases**: single

### Revision summary
Rewrote the plan around texture mapping per operator direction, replacing
the rejected per-cell weighted-accumulation kernel (Plan Review Finding 1:
mathematically inert). New shape:
- **A. Single pass**: per-pass, per-channel `SidescanTexture` in native
  (ping x sample) geometry; mesh stays at CUBE/bathy resolution; vertices
  carry UVs from the slant-range projection so GPU bilinear filtering
  blends across ping rows (the real cross-ping blend the review required).
  Subsumes the issue's deferred view-resolution-rasterization step.
  `PointCloudView::setSurface()`'s per-vertex-colour ceiling is addressed
  with a new additive `setSurfaceTexture()` entrypoint; issue #34
  (setSurface lacks buffer/index validation) is noted as adjacent, not
  fixed, and not worsened.
- **B. Composite**: per-pass offscreen ground-space FBO rendering with
  score written to depth (`GL_GREATER`) and amplitude to colour — highest-
  scoring pass's texel wins per texel, reproducing today's
  `ping_score * range_score` (both `RangeScoreMode` variants) and the
  "amplitude always beats shadow" rule via a low-but-nonzero shadow score.
- **C. Testability**: pure-CPU tests for the extracted `texel_score()` and
  box-average column builder; the two existing CPU composite tests
  (`StraightPassBeatsTurningPassInComposite`, `RangeScoreModeFlipsConflicts`)
  ported to GPU readback tests (precedent: `test_point_cloud_view.cpp`'s
  `grabFramebuffer()`/`MultiPassColour` pattern), not dropped; new tests for
  non-degenerate cross-ping blending, column-sharp shadow boundaries, and
  texture tiling past `GL_MAX_TEXTURE_SIZE`.
- Beamwidth convention/fallback (`tx_beamwidth_rad`,
  `resolve_reported_beamwidth()`) carried over from the prior plan, plus a
  **new plausibility-range guard** (reject `<=0` or `>1.2` rad) added
  mid-task per operator instruction: a third failure mode of
  `PingInfo`'s beamwidth field was found (degrees-vs-radians, in
  `cube_bathymetry/src/error_model.cpp:236-238`, dodged by the M3 driver
  leaving the field empty) alongside the two the issue already named
  (full-vs-half-extent, empty-vs-all-zeros). The driver comment's citation
  of "cube_bathymetry#30" as tracking this is corrected in the plan: that
  issue is closed and about a different subject (validating the error
  model against Calder's original) — the units bug is untracked.
- Plan Review Finding 3 (README wording) corrected in the new plan's
  Documentation & Instruction Impact section: a new subsection, not a
  correction of existing text.
- Open questions carried/updated: texture column pitch default, composite
  ground-texel default, non-uniform sample-count-per-pass handling
  (unverified against a real bag), shader-architecture micro-choice. The
  prior plan's triangular-vs-Gaussian kernel question is now moot — there
  is no hand-rolled along-track kernel in this design.

### Judged infeasible / descoped
- The prior plan's step 5 (decouple the drape grid cell from the CUBE
  surface cell via `extend_surface_for_drape()` resampling) is dropped
  entirely, not merely revised — texture resolution replaces it as the
  mechanism for imagery detail, and resampling the terrain itself is
  explicitly what the operator's redirection says not to do.

### Open question for the operator
- Whether Approach A (single-pass texture path) and Approach B (GPU
  composite) should split into two PRs once A's shader/texture plumbing
  exists as a concrete diff — left as an implementation-time judgment
  call in the plan (Estimated Scope) rather than decided now, since A.2's
  texture builder and A.5's shared ping-gate helper are load-bearing for
  both and a clean split isn't obvious until A is drafted.

**By**: Claude Code Agent (Claude Sonnet)

## Plan Review
**Status**: complete
**When**: 2026-09-03 14:01 -04:00
**By**: Claude Code Agent (Claude Opus)

**Issue**: #39 — Sidescan drape discards ~5 of every 6 across-track samples — nearest-sample resampling, not a resolution limit
**Plan file**: `.agent/work-plans/issue-39/plan.md`
**Branch**: `feature/issue-39`

**Plan**: `.agent/work-plans/issue-39/plan.md` at `a388281`
**PR**: PR-less
**Verdict**: changes-requested

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Needs work | Architecturally the redesign is right-sized for what it does, but at 10 files/file-pairs and 4 major components (beamwidth guard, texture builder, UV mesh, GPU shader+FBO pipeline) it exceeds the skill's own single-PR guidance; see Finding 5 (scope). |
| Issue alignment | Good | Directly fixes the issue's across-track nearest-sample loss via box-averaged texture columns + GPU bilinear filtering; folds in the deferred view-resolution step with a stated rationale. |
| File targeting | Good | Verified against source: no `sidescan_geometry.cpp` (header-only, correcting the prior plan), single `PointCloudView::setSurface()` per-vertex-colour ceiling confirmed (no `QOpenGLTexture`/GL texture calls exist in `src/` today), single call site shape for the composite path in `sidescan_viewer_window.cpp`. |
| Consequences | Good | All 5 Issue Review action items are covered; the gate-sync item is solved structurally (one shared `ping_is_drapable()` helper) rather than by comment-mirroring, which is a real improvement over the status quo the review flagged. |
| Documentation & instruction impact | Good | Non-silent, correctly revised (README needs a *new* subsection, not a correction — verified: `.agents/README.md` has no drape-kernel text today, only `sidescan_drape.hpp`'s header comment does). |
| Principle alignment | Needs work | "Fix completely" and "test what breaks" are well served architecturally, but two of the plan's own safety claims (the beamwidth guard, the shadow-score sentinel) are asserted without being derived/verified against the code that must make them true — see Findings 1 and 2. |
| ADR compliance | N/A | Correctly noted — no ADR governs sonar geometry/GPU rendering in this repo. |
| ROS conventions | N/A | No topics/params/messages changed. |

### Findings

1. **(must-fix) [Approach A.1 — beamwidth plausibility guard]** — `plan.md:183-198`. The single upper bound (`> 1.2` rad rejected) does not catch the failure mode it names as reason #3 for existing (`error_model.cpp:236-238` treating the field as degrees) when applied to the field this code actually consumes.

   `tx_beamwidth_rad` carries the **along-track** figure, whose true value in this issue's own ground truth is 0.00768 rad (0.44°). A producer that emits the field in degrees instead of radians for an along-track beam of this size would write a raw value of `0.44` — which is `<= 1.2`, so `resolve_reported_beamwidth()` accepts it as a legitimate radian value 57x too small (25.2° being read as 0.44 rad ≈ 25.2° worth of angular error is backwards — concretely: reading "0.44" as radians when it means "0.44°" makes the along-track footprint ~57x too wide, not too narrow). The plan's own worked justification (`plan.md:186-198`) derives the `1.2` bound from the widest **across-track** rx figure (55° = 0.96 rad) as "a ceiling reference" — but concedes in the same paragraph that `tx_beamwidth_rad` is a different, much smaller field, and never re-derives the bound against *that* field's plausible range. A magnitude-only upper bound cannot distinguish "0.44 rad, genuinely reported" from "0.44, mislabeled degrees" — both numbers are identical. This is not a corner case: it is precisely the along-track regime this issue's own measurements live in.

   Confirming this is a real gap, not just a theoretical one: the plan's own test entry (`plan.md`, Files to Change, `test_sidescan_texture.cpp`) proposes testing rejection with `25.0f` — a value chosen to trivially exceed `1.2` and say nothing about the actual danger zone (0–1.2 rad, which is exactly where a mislabeled small-degree value lands). No test in the plan exercises the case that matters.

   A magnitude bound alone cannot solve this — it needs either a much tighter, along-track-specific range derived from the device's own known geometry (with explicit false-reject risk stated), or a cross-check against independent information (e.g. the ratio against `rx_beamwidths` when both are reported, since a fan array's along-track figure should be much smaller than its across-track one; or a plausibility check against the ping-spacing-derived footprint the not-reported fallback already computes). Whichever is chosen, the plan needs to say which, because "reject `> 1.2`" as currently specified does not defend the field it is attached to.

2. **(should-fix) [Approach B.2 — shadow score sentinel]** — `plan.md:411-418`. The "very low but nonzero" shadow score (`1e-6`) is asserted to sit "safely below the real score floor of `ping_score_min * 0.05`" — but no such floor is enforced anywhere in the code this plan is porting. `ping_score = 1.0 / (1.0 + (rate / kRateHalf)^2)` (`sidescan_drape.cpp:433-451`, carried over unchanged per the plan) decays asymptotically toward 0 as yaw rate grows; nothing clamps it to a minimum. `range_score`'s floor is `0.05`, so the true score floor is `ping_score * 0.05` with no lower bound on `ping_score` itself. For an extreme-but-not-impossible yaw rate (e.g. a bad-GPS glitch producing a spurious large heading jump between consecutive pings), `ping_score` can fall arbitrarily close to `1e-6/0.05 = 2e-5` and below, at which point the GPU depth test would let a shadow mark from a *different* pass win over that cell's genuinely-painted (if very low quality) amplitude — a real amplitude losing to a shadow mark, the exact inversion of the "amplitude always beats shadow" rule the plan says B.2 reproduces "exactly." Today's CPU code has no such failure mode: the `!isfinite(out.amplitude[ci])` check in the paint branch is unconditional and never compares to `score` at all, so a real amplitude *always* overwrites a prior shadow mark regardless of how low its score is. Recommend either clamping `ping_score` to an explicit floor (with the clamp value chosen so the shadow sentinel is provably, not asymptotically, below it) or re-deriving the sentinel choice against that clamp — and stating the derivation in the plan rather than asserting an unclamped asymptote is "safely below."

3. **(suggestion) [Approach A.3 — UV interpolation across genuine near-range gaps]** — `plan.md:285-303`. Below the issue's own ~9 m regime change, the beam footprint (3.8–7.7 cm) is narrower than the ping-to-ping spacing (6.9 cm) — i.e. there are along-track ground positions no single ping's beam actually ensonified. The bracketing-pair UV scheme (A.3) does not distinguish this case from the "real cross-ping blend" case the plan is built around: for *any* along-track position between two consecutive same-channel pings, GPU bilinear filtering will synthesize a smoothly-interpolated value, whether or not the true footprints on either side actually overlap that position. This is likely a net improvement over today's behaviour (which strip-paints one sample verbatim across the same span, a blockier form of the same fabrication) rather than a regression, but the plan doesn't say so explicitly, and doesn't test for it — recommend either an explicit note that this is an accepted, no-worse-than-today limitation, or (if the operator wants it) a coverage/confidence channel analogous to the CUBE membrane-fill's `uncertainty = NaN` convention already used elsewhere in this repo for "terrain the drape lands on but wasn't directly measured." Related and also unspecified: "nearest bracketing pair within a **reasonable** along-track distance" (`plan.md:299-300`) has no numeric bound, so as written a recording gap (e.g. a paused/restarted session) could bracket-interpolate across an arbitrarily large blank stretch. Needs a concrete cutoff, analogous to `extend_surface_for_drape()`'s existing per-ping gates.

4. **(suggestion) [Approach C.2 — GPU readback assertion precision]** — `plan.md:445-458`. The ported tests are specified as asserting the conflict cell's colour "matches the straight pass's LUT colour" — this reads as closer to an exact/byte-level comparison than the workspace's own established precedent (`test_point_cloud_view.cpp`'s `MultiPassColour`, which asserts *which colour family dominates* a region via a threshold, not exact-match). GPU pixel readback carries genuine sources of imprecision (software-GL rasterization rounding, LUT interpolation, texture-filtering edge effects) that an exact-match assertion is more likely to catch spuriously. Recommend the plan specify the same threshold/dominance-style assertion the precedent uses, to avoid the ported tests being flaky (and to avoid a later implementer loosening them ad hoc without review).

5. **(suggestion) [Scope — one PR vs. two]** — `plan.md:131-136` states definitively "build A and B in one PR," but `plan.md:588-599` ("Estimated Scope") reopens the same question as an undecided "implementation-time judgment call" — the plan is internally inconsistent about whether this is settled. Substantively: A.2 says the `ping_score`/range-input computation **moves** (not copies) out of `drape_pass()` into the new texture builder (`plan.md:248-253`) — if `drape_pass()` loses that computation, the old CPU composite path (and its two existing tests) breaks the moment A lands, even before B exists, which is the real (but unstated) reason A and B are coupled. If that coupling is accepted, one PR is the right call and the plan should say so plainly instead of the vaguer "own tests need to port" framing at `plan.md:131-136`, and drop the reopened question at the bottom. If the coupling is *not* wanted, the cleaner path is to make A.2's extraction a **copy**, leaving `drape_pass()` and its composite consumer fully untouched until B lands, which would let A ship as a reviewable, revertable single-pass texture PR on its own (new file pair + new `PointCloudView` entrypoint + one consumer call-site rewire — a much more normal PR size) with B's GPU-compositing/FBO/shader work following as its own PR. Given the size here (10 files/file-pairs, four largely-independent hard problems: beamwidth plausibility, a new GPU texture/shader pipeline, a new UV mesh builder, and offscreen FBO compositing with a hand-transcribed GLSL scoring function), the two-PR split is the better call — it lets A's rendering/UV correctness get reviewed and land before the harder GPU-compositing correctness work (Findings 2–4 above) is layered on top of it, and gives a natural rollback point if A's texture approach turns out to need rework once tried against a real bag.

### Round-1 findings — verification (not just re-reading)

- **Finding 1 (must-fix, inert along-track weighted accumulation)**: genuinely resolved, not reworded. Verified by reading the new mechanism: UV rows are built one-per-ping from each ping's own real amplitude samples (`sidescan_texture.hpp`'s planned `SidescanTexture`, A.2), and `V` interpolates between **two different pings' real rows** (A.3) — this is categorically different from the old plan's per-offset weighting over one ping's single repeated value, which is what made the round-1 finding fatal. No trace of the old mechanism remains in the plan.
- **Finding 2 (suggestion, dilution-aware across-track test)**: carried forward correctly by reference ("folded into this plan's test plan directly rather than needing a second fix-up," `plan.md:243-247`); the box-average math is verbatim from the prior plan's accepted step 4. The new plan doesn't restate the quantitative acceptance criterion inline in the test-plan row — minor, low-priority polish, not a re-open.
- **Finding 3 (suggestion, README wording)**: correctly fixed — Documentation & Instruction Impact now says "new subsection," matching source (verified: no drape-kernel description exists in `.agents/README.md` today; only `sidescan_drape.hpp`'s header comment has it). One adjacent, self-inflicted accuracy slip in the *new* plan: `plan.md:92-94` claims `test_point_cloud_view` is "the **only** test target with that [`QT_QPA_PLATFORM=offscreen`] environment today" — verified false: `test_survey_explorer_window` (`CMakeLists.txt:359-360`) carries the identical `set_tests_properties(... ENVIRONMENT "QT_QPA_PLATFORM=offscreen;LIBGL_ALWAYS_SOFTWARE=1")`. Doesn't change the Files-to-Change list or any technical decision, but is worth a one-line correction since the plan's Principles Self-Check claims all references were "re-verified against current source."

### Summary

The revision genuinely fixes the fatal round-1 problem — the texture-mapped, cross-ping-bilinear architecture is a real, non-degenerate blend, correctly identified and well-reasoned against source. But two of the plan's own new safety claims don't hold up against the code they describe: the beamwidth plausibility guard (A.1) doesn't defend the along-track field it's attached to against the exact degrees/radians failure it names as motivation, and the GPU shadow-score sentinel (B.2) relies on an unenforced score floor to claim exact parity with today's CPU semantics. Both are fixable without touching the architecture. Scope is also worth reopening: the plan argues itself into "one PR" on a rationale (test porting) that doesn't hold once the real coupling (A.2's ping_score *move*) is identified, and a clean A/B split looks achievable and would reduce review risk on the harder GPU-compositing half.

### Recommended Actions

- [ ] Resolve Finding 1 — redesign or replace the beamwidth plausibility bound so it actually catches a degrees-mislabeled along-track value (the field's true operating range), not just large numbers; add a test in the danger zone (0–1.2 rad), not only above it.
- [ ] Resolve Finding 2 — either enforce and cite an actual `ping_score` floor, or re-derive the shadow sentinel against a proven bound; state the derivation in the plan.
- [ ] Address Finding 3 — state explicitly whether near-range cross-ping interpolation over genuine coverage gaps is an accepted limitation (no worse than today) or needs a confidence marker; give the bracketing-pair search a concrete along-track distance bound.
- [ ] Address Finding 4 — specify the composite readback tests' assertion style (dominance/threshold, matching `MultiPassColour`'s precedent) rather than implying exact colour match.
- [ ] Resolve Finding 5 — decide move-vs-copy for A.2's `ping_score` extraction and settle the one-PR-vs-two question consistently between the Scope paragraph and Estimated Scope; recommend two PRs (A, then B) given the size and the independence of A's correctness risk from B's.
- [ ] Minor: correct `plan.md:92-94`'s "only test target" claim (`test_survey_explorer_window` also carries the offscreen-GL environment property).

## Plan Authored
**Status**: complete
**When**: 2026-09-03 14:30 -04:00
**By**: Claude Code Agent (Claude Sonnet)

**Plan**: `.agent/work-plans/issue-39/plan.md` at `aaa0ffd`
**Branch**: feature/issue-39 at `aaa0ffd`
**Phases**: single

Round-3 revision of the texture-mapped sidescan drape plan, addressing the
round-2 `## Plan Review` (`a388281`) must-fix findings:

- Beamwidth plausibility guard: `resolve_reported_beamwidth()` now takes the
  plausible range as a parameter; `kAlongTrackMaxBeamwidthRad = 0.1` rad and
  `kAcrossTrackMaxBeamwidthRad = 1.2` rad replace the single global bound
  that let a degrees-mislabeled along-track value (0.44) pass through
  untouched. The along-track bound now rejects that exact confusion value
  with a 4.4x margin.
- Shadow sentinel: `ping_score` is clamped to `kPingScoreFloor = 0.01`,
  giving a provable composite-score floor of `5e-4`; `kShadowSentinelScore
  = 1e-4` sits below that floor by construction, so a real (however noisy)
  amplitude fragment cannot collide with the shadow sentinel even under the
  bad-gyro corpus case (2026-06-26 Massabesic bags).
- No-data preservation: amplitude texture sampling replaces hardware
  `GL_LINEAR` with manual, coverage-gated interpolation using a new
  per-row `row_footprint_v_halfwidth` field, so a near-range coverage gap
  (footprint narrower than ping spacing, inside ~9 m) renders as no-data
  rather than a fabricated cross-ping blend.
- Composite readback tests (C.2) now specified as dominance/threshold
  assertions following `MultiPassColour`'s precedent
  (`test/test_point_cloud_view.cpp:162-189`), not exact-colour matches.
- One-PR/two-PR inconsistency resolved per operator decision: Estimated
  Scope now asserts a single PR unconditionally; A.2's `ping_score` move
  (not copy) out of `drape_pass()` is explicitly tied to that decision.

Three new/changed tests added to the test plan directly answering the
must-fixes: `resolve_reported_beamwidth()` rejecting the real confusion
value `0.44` (not `25.0`), `ExtremeYawPingAmplitudeStillWins` (bad-gyro
shaped ping, asserts amplitude beats shadow), `CoverageGapStaysNoData`
(asserts a genuine near-range gap renders no-data, not blended).

### Open questions
- [ ] `texture_col_pitch_m` default — operator call.
- [ ] `composite_texel_m` default for the composite FBO — operator call.
- [ ] Non-uniform sample counts across a pass's pings — needs a source
  check against a real bag before implementation locks in "pad shorter
  rows" as sufficient.
- [ ] Second shader program vs. `u_use_texture` uniform switch —
  implementation-level choice, no behavioural difference.
- [ ] Whether `test_sidescan_composite.cpp` is its own file or folds into
  `test_point_cloud_view.cpp` — depends on fixture-code overlap, decide
  once both are drafted.

## Plan Review
**Status**: complete
**When**: 2026-09-03 14:35 -04:00
**By**: Claude Code Agent (Claude Opus)

**Plan**: `.agent/work-plans/issue-39/plan.md` at `aaa0ffd`
**PR**: PR-less
**Verdict**: approve-with-suggestions

### Evaluation

| Dimension | Verdict | Notes |
|---|---|---|
| Scope | Good | Single PR, A+B coupled, per operator decision; no longer reopened. |
| Issue alignment | Good | Subsumes issue's deferred step 3; addresses issue review's gate-sync action item via `ping_is_drapable()`. |
| File targeting | Good | Matches verified source layout (`sidescan_geometry.hpp` confirmed header-only, no `.cpp`). |
| Consequences | Good | Table covers construction sites, gate unification, `setSurface()` ceiling, CPU->GPU LUT move, test porting, `.agents/README.md`, issue #34 adjacency. |
| Documentation & instruction impact | Good | Non-silent; correctly scoped as new subsection, not a correction. |
| Principle alignment | Good | Test-driven, fix-completely, consequences-included all addressed with specifics. |
| ADR compliance | N/A | No project ADRs exist; correctly stated. |
| ROS conventions | N/A | Not a ROS-interface change. |

### Findings

Round-2 must-fixes verified against source, not just re-read:

1. **Must-fix 1 (beamwidth axis) — genuinely fixed for along-track.** Confirmed `resolve_reported_beamwidth()` is not yet in source (still to be added) and the plan's derivation is sound: `kAlongTrackMaxBeamwidthRad = 0.1` rejects the real confusion value `0.44` (4.4x margin) while admitting the measured `0.00768`. Verified independently: no along-track degrees value in the sensor class's plausible range (single-digit-to-tens of degrees, i.e. numerically 0.09-0.87 in raw form) survives below `0.1` except implausibly-narrow beams under ~5.7°, which is an acceptable edge for this guard's stated purpose.

   **Residual gap, not must-fix**: for the across-track axis, `kAcrossTrackMaxBeamwidthRad = 1.2` cannot distinguish degrees-mislabeled-as-radians from a genuine radians value, because real rx beamwidths (tens of degrees, e.g. 20-68°) numerically overlap the plausible-radians range (0-1.2) when misread as bare degrees — e.g. a hypothetical 0.9°-beam sensor's degrees value (0.9) misread as radians (0.9 rad = 51.6°) is itself a physically plausible radians value, so the guard cannot flag it as wrong by range alone. This is structurally different from the along-track case, where the true-radians range and the degrees-mistake range are well separated. The plan doesn't claim otherwise for across-track (it only says "sits above the measured value with headroom," not "catches the degrees confusion"), and the field isn't currently consumed by the drape kernel, so this isn't a must-fix — but the plan should say so explicitly (one sentence) rather than leaving a reader to assume symmetric protection from the parallel presentation of both constants.

2. **Must-fix 2 (shadow-sentinel floor) — genuinely fixed, arithmetic verified against source.** Confirmed in `src/sidescan_drape.cpp`: `range_score`'s `std::max(0.05, ...)` floor (line ~191) is unconditional across both `RangeScoreMode` variants — it applies before the ternary, not after. Confirmed `ping_score = 1/(1+(rate/0.05)^2)` (line ~449) is genuinely unclamped in the current code and decays toward but never reaches 0 as `rate` grows — matches the plan's claim exactly, formula and line numbers both check out. The clamp point (A.2, the texture builder) is the single place `ping_score` is computed under this plan, and B.2's `texel_score()` consumes only that clamped value, so `kPingScoreFloor * range_score_floor = 5e-4` is a genuine, provable floor, not an inspection-based guess. `kShadowSentinelScore = 1e-4` sits below it by construction. No path bypasses the clamp as designed.

3. **Clamp is a behaviour change — plan owns it, but slightly undersells how easily it's reached.** Verified against the existing `StraightPassBeatsTurningPassInComposite`/`RangeScoreModeFlipsConflicts` tests (`test/test_sidescan_drape.cpp`): neither depends on unclamped ordering — the turning-pass fixture in the first test already computes a raw `ping_score` of ~0.0039 (rate = 0.8 rad/m from `yaw += 0.4*i` over 0.5 m ping spacing), which is *already below* the proposed `kPingScoreFloor = 0.01`, i.e. this existing fixture already sits in clamp territory today under the new code — yet the test's assertion (straight beats turning) holds identically whether the turning pass's score is 0.0039 or clamped to 0.01, since 1.0 overwhelms either. So: no test breaks. But this means the clamp engages for what the test's own comment calls an ordinary "mid-turn" scenario (not sensor-pose noise), which is milder framing than the plan's "genuinely violent turn ... not normal survey track-keeping." Not a correctness problem — the clamp's job (bound the floor) is unaffected by how often it engages — but the plan's characterization of when it engages is optimistic. Suggest tightening that one sentence in B.2, or noting the existing turning-pass fixture as a concrete example of legitimate track-keeping that already reaches the clamp.

4. **Must-fix 3 (coverage-gated interpolation) — mechanism is sound and `CoverageGapStaysNoData` has real discriminating power.** The three-case logic (both-cover / one-covers / neither-covers) is well-defined and the "neither covers -> no-data" case is exactly what a hardware `GL_LINEAR` sampler cannot express (it always blends), so the test's stated "fails against GL_LINEAR by construction" claim holds. Boundary behavior at the very first/last row of a pass (where only one candidate row exists on one side) isn't spelled out in prose, but the described mechanism degrades correctly there by inspection (one-sided coverage collapses to the existing "exactly one row covers" branch, and true off-pass fragments are already caught by A.3's out-of-range UV sentinel) — this is a documentation completeness nit, not a design gap.

5. **Fix 4 (test citation) — confirmed accurate.** `test/test_point_cloud_view.cpp:162-189` is exactly `MultiPassColour` (`TEST_F` opens at line 162, next test `ColorModePassOnSinglePassIsSafe` opens at line 192). Its dominance-predicate assertion style (per-channel ratio thresholds, not exact `QColor` match) is real and does generalize to the planned composite tests as described.

6. **Open questions — correctly triaged.** `texture_col_pitch_m` and `composite_texel_m` defaults are genuine operator-facing fidelity/perf tradeoffs, appropriately left open. Shader-program-vs-uniform-switch and the test-file split are implementation-level with no behavioral difference, correctly deferred rather than escalated. Non-uniform sample counts is a factual question needing a real-bag check, not a preference — the plan frames it that way already ("needs checking... before implementation locks in"), which is right; only miss is not naming who/when does that check (implementer, at A.2 implementation time, presumably) — minor.

### Summary

Round-2's four must-fix findings are genuinely resolved, not reworded: the beamwidth bound derivation, the provable score floor, the coverage-gated interpolation design, and the test citation all check out against the actual source (`sidescan_drape.cpp`, `sidescan_geometry.hpp`, `test/test_point_cloud_view.cpp`) rather than merely against the plan's own prose. The plan is ready for implementation. Two non-blocking suggestions carried forward as follow-up polish, not gates: (a) state explicitly that the across-track beamwidth bound doesn't distinguish degrees/radians confusion the way the along-track bound now does (it protects only against implausibly-large values), and (b) soften "genuinely violent turn ... not normal survey track-keeping" in B.2, since the existing turning-pass test fixture already reaches clamp territory under ordinary mid-turn conditions.

### Recommended Actions

- [ ] (suggestion) Add one sentence to A.1 noting the across-track bound (`kAcrossTrackMaxBeamwidthRad`) does not disambiguate degrees-vs-radians the way the along-track bound does, since real across-track values and their degrees-misread equivalents overlap numerically — acceptable given the field is currently unconsumed, but should be stated rather than left implicit.
- [ ] (suggestion) Soften B.2's claim that the clamp "only changes behaviour for exactly the pathological input class" — the existing `StraightPassBeatsTurningPassInComposite` fixture's turning pass already computes a raw `ping_score` (~0.0039) below the clamp floor under ordinary mid-turn conditions, though no test assertion depends on the unclamped value.
- [ ] Proceed to implementation; no must-fix findings remain.
