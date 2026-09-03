---
issue: 26
---

# Issue #26 — Time bar: local-time display + dates in day-level tick labels

## Integrated Review
**Status**: complete
**When**: 2026-09-03 10:10 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))

**PR**: #31 at `9eae52e`
**Sources**: 4 (Copilot R1 @ `6b2fbf0a`, Copilot R2 @ `bc22034`, Copilot R3 @ `9eae52e`, CI rollup). No prior local-review timeline — this PR came from an interactive desk session, not the plan-first lifecycle, so there are no cross-source confirmations to report.
**Cross-source confirmations**: 0
**CI**: all-pass (`build-and-test` success, `copilot-pull-request-reviewer` success)

### Findings
- [ ] (medium, Copilot R3 suppressed) `extend_surface_for_drape()` admits pings that `drapePing()` will always skip — its usability filter checks `metres_per_sample`, `lateral_sign` and `amplitudes.empty()` but NOT `altitude`, while `drapePing()` gates on `g.altitude <= 0.0` (sidescan_drape.cpp:84). An altitude-less ping therefore grows the terrain grid it can never paint. Not cosmetic: growth is capped by `max_nodes` and shrunk proportionally on all sides, so an un-drapable ping can steal grid budget from drapable ones and degrade the real drape. Fix: add `g.altitude <= 0.0` to the filter at sidescan_drape.cpp:220-224 — `altitude` is a pure ping property, checkable before extension, unlike the nadir-surface condition which is chicken-and-egg. — `src/sidescan_drape.cpp`
- [ ] (low, Copilot R1) The conflict-score comment claims "single-pass conflicts reduce to nearer-wins", which is false ACROSS pings within a pass: `score = ping_score(straightness) * range_score`, and straightness varies per ping, so a farther sample on a straighter ping outranks a nearer one on a turning ping. The claim holds only WITHIN a single ping, where `ping_score` is constant. Fix: say "within a ping" (docs-only; the scoring itself is the intended behaviour). — `src/sidescan_drape.cpp`
- [ ] (low, Copilot R1) Same root cause in the public header: the `Nearest` bullet says closer samples "strictly outrank" far ones. True only within a ping; also worth recording that `range_score` is clamped at 0.05, so far-range differences are compressed rather than unbounded. Fix with the sibling comment in one commit. — `src/sidescan_drape.hpp`

### False positives
- (Copilot R2 suppressed) "`rec.depth_var` strongly implies a variance (m²) ... will mis-scale the uncertainty shade/export by a square." Verified against the library, not the field name: `cube_bathymetry/include/cube_bathymetry/node.h:53` documents `depth_var` as "Depth uncertainty (confidence-scaled stddev)", and `node.cpp:286`/`:293` assign `stddev_to_confidence_interval_scale * std::sqrt(hypothesis->input_sample_variance)` on BOTH hypothesis paths. The value is already a confidence-scaled standard deviation in metres, so the `uncertainty_m` band name and the UI shade are correct and no sqrt is warranted — applying one would introduce the very error the comment warns of.
- (Copilot R2 suppressed, second half) "can also leak non-NaN uncertainty values for nodes with no depth estimate." Cannot occur: `extractNodeRecord()` sets `depth` and `depth_var` together inside each hypothesis branch, and the `if(!chosen)` path returns the default-constructed record with both fields still `std::nan("")` (node.cpp:275-300). There is no path that writes one without the other.

### Note (not a finding on this PR)
- The R2 false positive was caused by a genuinely misleading name upstream: `cube_bathymetry`'s `NodeRecord::depth_var` holds a standard deviation, not a variance. A reviewer reading the field name rather than its doc comment will reach the wrong conclusion — as Copilot did. Worth raising against cube_bathymetry (adjacent to, but distinct from, cube#99's node/BAG terminology scope). Not actionable here.
