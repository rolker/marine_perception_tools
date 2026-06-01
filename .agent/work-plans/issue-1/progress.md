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
