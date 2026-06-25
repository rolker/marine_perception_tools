---
issue: 8
---

# Issue #8 — Offline viewer: MBES 3D point cloud + backscatter waterfall + water-column echogram (multi-pane, shared widgets)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-24 17:57 -0400
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: approved

**Branch**: feature/issue-8 at `297c31c`
**Mode**: pre-push
**Depth**: Deep (reason: 1425+/409- across 16 files; GPU/OpenGL, off-thread bag I/O, TF)
**Must-fix**: 1 (fixed) | **Suggestions**: 6
**Round**: 1 | **Ship**: recommended — cross-confirmed must-fix fixed; remainder are documented edge-case limitations, not blockers

### Findings
- [x] (must-fix) MBES backscatter waterfall lacked `set_history` → dense (>200-ping) window evicted oldest rows, out of lockstep with other panes — `sidescan_viewer_window.cpp` onRenderFinished (fixed 297c31c)
- [x] (suggestion) sidescan rows ignored `sample0` near-gate → lib pane's linear-from-zero range axis + pixel→map marking regressed vs retired SidescanWaterfall (June 15 data sample0=14, ~0.13 m); now prepends gate samples — `build_sidescan_rows` (fixed 297c31c)
- [x] (suggestion) `palette_combo_` dereferenced without the null guard used in requestRender — `onRenderFinished` (fixed 297c31c)
- [ ] (suggestion) `render_window` early-returns when the sidescan window is empty, skipping MBES/down reads → 3D/backscatter/echogram blank on a sidescan dropout that still has MBES data — known limitation (sidescan+MBES co-occur in survey bags)
- [ ] (suggestion) EchogramWidget has no clear(); scrubbing into a region with no down-channel pings leaves the prior window's curtain (stale) — `onRenderFinished`
- [ ] (suggestion) stamp→slot maps in `readMbesWindow`/`readDownImages` keep only the first ping on a duplicate `stamp_ns` (rare) — `sidescan_bag_session.cpp`
- [ ] (suggestion) port/stbd paired by along-track index in `build_sidescan_rows` (carried-over assumption; shifts at the unpaired tail when counts differ)

### Verified sound (no action)
- `mbes_geometry::project_beam` matches the documented cube formula (x=-r·sin tx, y=r·sin rx, z=r·cos tx·cos rx); skips non-positive twtt/sound_speed; bounds-checked per beam.
- Threading: no worker touches a Qt widget or shared mutable state; render worker reads through the kept-alive session shared_ptr and returns a copyable POD; epoch stale-drop + destructor waitForFinished correct.
- PointCloudView GL lifecycle guarded (gl_ready_, initializeOpenGLFunctions check, makeCurrent/doneCurrent teardown); setters stage CPU state and defer GL via update().
- SidescanWaterfall retirement clean: no dangling refs to the deleted symbols/headers in src/, test/, or CMake; boxMarked rewired to the lib widget.

### Note
Static analysis run via `colcon test` (ament_lint included): 228 tests, 0 failures.
Two disjoint-lens Claude adversarial passes (Lens A logic / Lens B systemic); Copilot off (opt-in, suspended through June 2026).

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-06-25 07:40 -0400
**By**: Claude Code Agent (Claude Opus 4.8)
**Verdict**: approved

**Branch**: feature/issue-8 at `e45f170`
**Mode**: pre-push
**Depth**: Deep (reason: 2617+/539- across 19 files, 22 commits; new threading model + GPU/GL)
**Must-fix**: 0 | **Suggestions**: 2
**Round**: 2 | **Ship**: recommended — threading model verified sound; remaining items are edge-case/throughput follow-ups

Covers the work since the Round-1 review (layout, per-window colormaps, global keys, 3D
fixes, colormap unification, G/M across-track backscatter + marking, H/J linked cursor +
seek, F streaming index). Two disjoint-lens adversarial passes (logic / concurrency).

### Findings
- [ ] (suggestion) Per-pane stationary cap divergence: when a window exceeds max_pings (600), readWindow/readMbesWindow/readDownImages each trim oldest independently, so the panes can drift slightly out of along-track lockstep — `sidescan_bag_session.cpp` (only on dense/stationary >600-ping windows; documented limitation, follow-up: shared distance cutoff)
- [ ] (suggestion) publishSnapshot deep-copies pings_/mbes_pings_ each ~1.5 s flush; cost grows toward the end of a long index — `buildIndex` (acceptable for a one-time load; follow-up: copy only the new tail or size-cap flush cadence)

### Verified sound (no action)
- F threading: no reader path touches the worker's mutable members; all accessors go through the immutable snapshot; publish→swap under snap_mutex_ is race/UAF/deadlock-free; cross-thread Q_EMIT indexProgress is safe (destructor waits index_watcher_ before ~QObject discards queued events); epoch guards correct for open-during-index; render worker captures the session shared_ptr.
- F correctness: final pass recomputes cumdist + sort + altitude authoritatively → matches pre-streaming result (probe: 616,568 detections / 143,305 soundings-per-50m / 234 beams / z=46.85 / 19,473.7 m). MbesPing tf_ok vs has_pose two-stage semantics clean.
- G: across-track sign = left-of-heading (port), nadir_index/range_max_port/stbd match the lib's WaterfallRow consumption; bin math + hold-fill + single-side handling correct.
- H/J: echogram frac↔distance mapping monotonic/consistent; nearest/positionAtDistance correct; span>0 guards.
- Boat arrow z (ENU z-up: max sounding z + lift floats above the cloud); paintGL depth-state reassert recovers the QPainter overlay's state before the depth clear; PointCloudView GL lifecycle guarded.

### Note
Static analysis run via colcon test (ament_lint): 228 tests, 0 failures. Copilot off (opt-in, suspended).
Lib dependency merged: marine_sonar_widgets#9 (jazzy); viewer rebuilt against the merged lib (no overlay).
