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
**When**: 2026-05-29 10:58 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-1/plan.md` (revised)
**Why**: discovered `unh_marine_perception#22` (plan PR #25) replaces the tuner's parameter model. Re-scoped to stacked Milestone A (algorithm-agnostic pipeline + viewer + #22-stable knobs, buildable now) / Milestone B (#22 knob set, after #22-P1). Param dock built table-driven so B is a data edit. See plan `## Implementation Notes`.

## Plan Review
**Status**: complete
**When**: 2026-05-29 11:01 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context)) (in-context — author self-review)

**Plan**: `.agent/work-plans/issue-1/plan.md` at `cf7201a`
**PR**: https://github.com/rolker/marine_perception_tools/pull/2
**Verdict**: approve-with-suggestions

### Findings
- [x] (must-fix) `res`/`half_extent` are `OccupancyBuffer` construction-time geometry, not `setParams`-able — engine must reconstruct on change, or make them CLI-only for Milestone A (recommended). — `plan.md` Approach step 3–4
- [x] (suggestion) `OccupancyBuffer::validate()` covers only `OccupancyParams`; `AccumulateParams` knobs (max_range, min_grazing_angle, res, half_extent) need the engine's own bounds-check (mirror driver `val>0`). — `plan.md` step 3/4
- [x] (suggestion) Fidelity wording: `accumulate_frame` is boat-centred (matches the #23 offline core / bag_to_costmap_video), not the camera-centred live SeaSurfaceLayer; soften "exactly the live accumulation order" / "offline result matches the boat". — `plan.md` step 3, Principles
- [x] (suggestion) `render_new`/`colour_logodds` are NOT exported (tool anonymous namespace) — tuner copies ~15 lines of rendering; clarify "algorithm reused, rendering reproduced" + palette-drift note. — `plan.md` Context/step 3
- [x] (suggestion) Note per-frame TF-lookup-failure handling (early stamps; mirror driver) and `plane_z=0` in `bizzy/map_tide`. — `plan.md` step 2

## Integrated Review
**Status**: complete
**When**: 2026-06-01 08:30 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #2 at `7c21e85`
**Sources**: 2 (Copilot R1 @ `7c21e85` current, Copilot R0 @ `0d7e3008` stale) + CI rollup. No local Local Review / Pre-Push timeline recorded.
**Cross-source confirmations**: 2 (raised by both Copilot rounds)
**CI**: all-pass (build-and-test ✓)

### Findings
- [ ] (cross-confirmed) Param dock re-sims on every `QDoubleSpinBox::valueChanged` — expensive replays / UI freeze on large windows; use Apply/editingFinished — `src/main_window.cpp:207`. **Already designed as D5 (Apply/Reset + dirty-colour); no action until D5.**
- [ ] (cross-confirmed) `--end-s` default `-1` loads/decodes the whole bag → unbounded memory / OOM on long 4-cam bags — `src/main.cpp:66`. **Already designed as D4 (windowed File→Open); confirmed live (OOM'd the machine this session); fixed by D4.**
- [ ] (minor, Copilot R1) `BagSession`/`loadWindow` comment claims "seeks … no re-open, no re-scan" but it re-opens a Reader and linearly filters by recv_timestamp — reword to match D2 reality (`Reader::seek` is the future optimization) — `src/bag_loader.hpp:120`.
- [ ] (minor, Copilot R1) Empty-window error prints negative upper bound (`"[0, -1]s"`) when `end_s<0`; format as "end" — `src/bag_loader.cpp:358`.
- [ ] (minor, Copilot R1) Status bar shows raw epoch `currentStamp()`; show bag-relative time — `src/main_window.cpp:316`. **Folds into D4 (time-based scrubber introduces bag-relative time).**
- [ ] (latent, Copilot R0) Ctor calls `accumulate(0)` unconditionally; an empty `LoadedBag` reads past `frames[0]`. No live caller (load_bag/BagSession throw on zero frames) but D4 buffer manager or future callers could — add an empty-frames guard — `src/resim_engine.cpp:76`.

### False positives
- (Copilot R0) "ctor throws → needs `<stdexcept>`" — the ctor doesn't throw; `BagSession`/`load_bag` throw in `bag_loader.cpp`, which already includes `<stdexcept>` (line 26). Premise never implemented; builds clean.
- (Copilot R0) Scrubber range underflow when `frameCount()==0` — guarded: `setRange(0, have ? frameCount()-1 : 0)` yields 0 with no engine, and `load_bag` guarantees ≥1 frame when an engine exists, so `frameCount()-1 ≥ 0`. The `-1` path is unreachable.
- (Copilot R0) `arg_double` uses `atof` (silent 0.0 on bad input) — real but deferred: `main.cpp` CLI parsing is reworked in D4 (new `--integration-halflives`/`--margin-s`/`--retention-s`); harden the parse there rather than as a standalone fix.

## Integrated Review
**Status**: complete
**When**: 2026-06-01 13:05 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #2 at `01cb273` (head `54e418c` is docs-only; latest code reviewed is `01cb273`)
**Sources**: 1 (Copilot R3 @ `01cb273`, the run-feedback review) + prior Integrated Review @ `7c21e85` + CI rollup. No local pre-push review at this head.
**Cross-source confirmations**: 0
**CI**: all-pass (build-and-test ✓ 4m33s)

### Findings
- [ ] (major, Copilot R3) half-life↔integration coupling broken on Apply: `integrationSeconds()` reads `engine_->occupancyParams()` not `applied_occ_`, and `onApplyParams` reuses `current_bag_` even when `decay_half_life_s` changed — a larger half-life needs a wider warm-up window but Apply re-warms the old (too-short) one → under-warmed costmap. Fix: derive integration from `applied_occ_`; on Apply, reload when the new required window exceeds the loaded span — `src/main_window.cpp:236,686`.
- [ ] (major, Copilot R3) render extent tied to `acc_.res`, not the buffer window: `renderGrid`/`renderRecorded` map pixel→world via `(u−panel_px/2)*acc_.res`, so a `panel_px` panel covers `panel_px·res` m — correct only by the coincidence panel_px(480)·res(0.25)=120m=2·half_extent at defaults. Non-default `--res`/`--window-m` mis-sample (can query outside the buffer). Fix: map over `[−half_extent,+half_extent]` (step `2·half_extent/panel_px`) — `src/resim_engine.cpp:390,421`.
- [ ] (major, Copilot R3) `openBag` doesn't guard an in-flight background load: swaps session_/resets engine_ and starts a new load while a prior QtConcurrent job may still run on the same watcher (setFuture mid-flight orphans it; a stale result could install against the new session). Fix: cancel/wait for the in-flight load before swapping (mirror ~MainWindow waitForFinished) and/or guard install by session identity — `src/main_window.cpp:206`.
- [ ] (minor, Copilot R3) BufferPlan Extend optimization unimplemented: `loadWindowJob` always `loadWindow(cache_lo,cache_hi)`, ignoring `plan.action`/`read_lo`/`read_hi` — the "extend reads only the new slice" path (16 assertions in test_buffer_policy) never runs; every reload full-reads from bag start (no Reader::seek). Not a correctness bug. Fix: implement incremental extend, OR document the policy-vs-loader gap so the tests don't imply a shipped optimization (document now, implement later) — `src/main_window.cpp:367`.
- [ ] (nit, Copilot R3) `RejectsInvalidOccupancyParamsWithoutStateChange` asserts hard-coded `obstacle_clamp == 5.0` (brittle to upstream default changes); the grids_equal(before,after) assertion already covers "no state change". Capture the pre-call value instead — `test/test_resim_engine.cpp:244`. Bundle with the half-life fix.

### False positives
- (none) — Copilot R3's findings are all valid or minor-valid; the test-brittleness item is real (kept as a nit), not dismissed.

## Integrated Review
**Status**: complete
**When**: 2026-06-01 13:50 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #2 at `51a06f6`
**Sources**: 3 (Copilot R4 @ `51a06f6`, prior Integrated Reviews @ `7c21e85`/`01cb273`, CI rollup)
**Cross-source confirmations**: 2 (Copilot vs. the repo's own documented contracts)
**CI**: all-pass (`build-and-test`)

### Findings
- [ ] (cross-confirmed: Copilot R4 + `buffer_policy.hpp:56-57` contract) `--start-s/--end-s` clamp the session (loader honors at `bag_loader.cpp:251-254`) but the UI ignores it — scrubber range `[0,duration]`, initial load `requestCoverage(0.0)`, `bufferParams()` hard-codes `bag_lo=0`/`bag_hi=duration`; scrubber disagrees with silently-clamped display — `src/main_window.cpp:241,258,276-277`. Fix: thread start_s/effective end_s into scrubber range + initial requestCoverage + bag_lo/bag_hi.
- [ ] (major, Copilot R4) In-window fast-path seek returns without recording the new target; a completing in-flight load installs the older target and overwrites the user's newer scrub (contradicts README "newer scrubs supersede") — `src/main_window.cpp:289-309`. Fix: set `chase_target_s_ = t_s` in the fast path when `load_in_flight_`.
- [ ] (cross-confirmed: Copilot R4 + the codebase's own applied_-not-engine_ pattern) Dock `Knob::read` getters read `engine_` not `applied_occ_`/`applied_acc_`; stale dirty-tracking/Reset during async Apply/window-warm — `src/main_window.cpp:495,512`. `integrationSeconds()` (267) already fixed for this exact pattern. Fix: read from applied_* structs.
- [ ] (minor, Copilot R2) `arg_double` uses `std::atof` (silent 0.0 on parse error / trailing garbage) — `src/main.cpp:35`.
- [ ] (minor, Copilot R3) `latestRgb` scans the whole `rgb_frames` vector for a sparse/absent camera every refresh (break gated behind cam filter) — `src/resim_engine.cpp:191`.
- [ ] (nit, Copilot R3) `camera_models` "always size kNumCameras" comment overstates the enforced contract — `src/bag_loader.hpp:101`.
- [ ] (nit, Copilot R3) Test asserts hard-coded `obstacle_clamp == 5.0` (brittle); `grids_equal` already covers no-state-change — `test/test_resim_engine.cpp:247`.

### Resolved since prior triage (verified fixed at `51a06f6`)
- integrationSeconds reads applied_occ_ (267); onApplyParams reloads wider window when half-life grows (724-735); openBag in-flight guard bumps request_id_/waitForFinished before session swap (220-228); `<stdexcept>` included (21); ReSimEngine ctor throws on empty LoadedBag before accumulate(0) (152-157); renderGrid/renderRecorded map via `2.0*acc_.half_extent/panel_px` (472,501); end_s<0 error prints "end" not "-1s" (386-387).

### False positives / non-issues
- (Copilot R1/R2) `--end-s -1` decodes the whole bag → OOM: the GUI path is windowed via `loadWindow`/`plan_buffer` (bounded memory); the full-range one-shot `load_bag` runs only under the explicit `--probe` headless diagnostic. Addressed by D4 windowing; default interactive open no longer OOMs.
- (Copilot R3) BufferPlan Extend ("reads only the new slice") unimplemented: documented deferral (`loadWindowJob` comment 391-398 + `.agents/README`) — a full re-read is a correct superset, not a defect; the policy/tests are staged for the future optimization.

## Integrated Review
**Status**: complete
**When**: 2026-06-02 09:15 -04:00
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**PR**: #2 at `636dcb2`
**Sources**: 4 (Copilot R5 @ `636dcb2`, prior Integrated Reviews @ `7c21e85`/`01cb273`/`51a06f6`, CI rollup)
**Cross-source confirmations**: 1 (`latestRgb` early-exit — Copilot R3 + R4 + R5, still open at this head)
**CI**: all-pass (`build-and-test` ✓ at `636dcb2`)

### Findings (all fixed this round)
- [x] (cross-confirmed: Copilot R3/R4/R5) `latestRgb` scans the whole globally-stamp-sorted `rgb_frames` for a sparse/absent camera every repaint — the `break` was gated behind the `cam` filter. Fixed: test `stamp_s > t` before the cam filter (behaviour-preserving early-exit) — `src/resim_engine.cpp:187`. New test `LatestRgbSelectsPerCameraFrameAtOrBeforeStamp` locks the contract.
- [x] (valid, Copilot R5) `rgbHorizon` shares the identical O(N) cam-gated scan — same reorder applied — `src/resim_engine.cpp:206`.
- [x] (minor, Copilot R5) `loadWindow` produced a confusing "no usable segmentation frames" error for an inverted window (`--end-s < --start-s`). Fixed: explicit precondition throw naming the real cause before any I/O — `src/bag_loader.cpp:256` (a negative `win_end` = "to end of bag" is exempt).

### Carry-forward (prior-round open, NOT re-flagged by R5, still unresolved at `636dcb2`)
- [ ] (minor, Copilot R2) `arg_double` uses `std::atof` (silent `0.0` on parse error / trailing garbage) — `src/main.cpp:32`.
- [ ] (nit, Copilot R3) `camera_models` "always size kNumCameras" comment overstates the enforced contract — `src/bag_loader.hpp`.
- [ ] (nit, Copilot R3) Test asserts hard-coded `obstacle_clamp == 5.0` (brittle); `grids_equal` already covers no-state-change — `test/test_resim_engine.cpp`.

### Resolved since R4 (verified at `636dcb2`, commit "honor session clamp + applied state")
- `Knob::read` getters now read `applied_occ_`/`applied_acc_` not `engine_` (519-540) — R4 cross-confirmed stale-dirty-tracking finding; `--start-s/--end-s` session clamp threaded into scrubber range + `bufferParams`; in-flight fast-path chase-target handled.

### False positives / non-issues
- (none this round) — all three R5 findings valid; the two perf items are low-severity but correctness-preserving and worth the O(N)→early-exit win on a per-repaint path.
