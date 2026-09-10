---
issue: 42
---

# Issue #42 — survey_explorer: one region for every action — left-drag selects, middle-click centres, modifiers retire

## Local Review
**Status**: complete
**When**: 2026-09-10 08:29 -04:00
**By**: Claude Code Agent (Claude Opus 5 (1M context))
**Verdict**: changes-requested

**PR**: #50 at `6e288b1`
**Mode**: post-PR
**Depth**: Deep (reason: user override; 8.1k-line diff across 54 files, cross-module UI + numerics)
**Must-fix**: 9 | **Suggestions**: 15

Completes the 2026-09-09 review, whose two adversarial passes were lost when
that session ended. The governance specialist's findings (posted as a PR
comment at the same head sha) are carried forward here; static analysis was
skipped (`--skip-static`) because pre-commit was already verified clean over
the whole branch.

### Findings
- [ ] (must-fix) Every uncertainty component is floored at 0.05 m, so the new angular weighting is inert below ~7 m depth and the operator-facing caveat states the ordering unconditionally — verified: ratio 1.000 at 2/5/7 m, 1.136 at 8 m, 1.420 only at >=10 m — `src/sounding_uncertainty.hpp:182,233-241`
- [ ] (must-fix) The test cited as the change's evidence does not isolate the angular term; vertical variance spans only 1.16x across its swath while absolute variance fell 9.4x, so a constant-sigma model would likely pass it — `test/test_cube_lab.cpp:620-661`
- [ ] (must-fix) The world-lift fix leaves its cause in place in both copies: both sites rebuild the sounding field by field from a default-constructed value, so any field added later is silently dropped again, and the consumer now drops such soundings outright — `src/mbes_window_reader.cpp:211-223`, `src/sidescan_bag_session.cpp:979-991`
- [ ] (must-fix) Cancellation covers teardown but never supersede: no in-flight guard on the cloud, CUBE or drape dispatch, so holding the clip-margin spin starts ~20 concurrent multi-bag loads that run to completion on the shared global thread pool — `src/sidescan_viewer_window.cpp:3825,1343,1874`
- [ ] (must-fix) The drape and cloud workers lack the top-level catch the index and CUBE workers have; a bad_alloc from the operator-invited "Run anyway" override rethrows on the UI thread or out of the destructor and terminates — `src/sidescan_viewer_window.cpp:1874-1926,3825-3830`
- [ ] (must-fix) (governance) The layer-name comment misreads uma-ADR-0007 A.2 and uma-ADR-0010 D3: `survey` is the correct current name and the recorded rename obligation must not be discharged — `src/world_layout.hpp:63-70`
- [ ] (must-fix) (governance) The paint-order fix is cited to uma-ADR-0013 D3 in three places; the decision that requires it is D5's ascending guarantee — `src/basemap_lod.cpp:380-390`, `src/basemap_lod.hpp:28-31,92-95`
- [ ] (must-fix) (governance) The interaction rework is unrecorded; the design doc still describes tile-selection auto-load — `docs/survey_explorer.md`
- [ ] (must-fix) (governance) Agent guide stale in four verified places (stores default, ctrl-click selection, auto-load, declutter toggles) and its layout tree omits nine new headers — `.agents/README.md`
- [ ] (suggestion) `abandonRecenter()` never emits `viewChanged()`, so an interrupted glide leaves the basemap holding the pre-glide viewport's tiles — `src/sidescan_canvas.cpp:220-229`
- [ ] (suggestion) Track rows are not checked for finiteness while the cursor is; one non-finite fix poisons the whole nearest-track search — `src/nav_track_hit.hpp:90-103`
- [ ] (suggestion) The floor on `sigma_y` is unreachable (`sigma_y >= 0.2` always) but reads as an active guard — `src/sounding_uncertainty.hpp:219-221`
- [ ] (suggestion) Grid extent and `soundings_in` are computed over soundings the new geometry guard then drops, so outliers can trip the max-nodes abort — `src/cube_lab.cpp:208,216-232`
- [ ] (suggestion) A bare relative `--index` filename now yields an empty stores dir and a silently basemap-less session — `src/survey_explorer_main.cpp:222-228`
- [ ] (suggestion) `cancelWorkers()` misses the basemap worker, whose cancel is only set during QObject teardown after the four waits — `src/sidescan_viewer_window.cpp:2577-2585`, `src/basemap_lod.cpp:201-209`
- [ ] (suggestion) `~BasemapLod` waits only the current future while `open()` sets a future with no isRunning guard, so a superseded pass can outlive the object — `src/basemap_lod.cpp:206-208,235`
- [ ] (suggestion) The cache temp name is per-process, not per-thread; uncancelled superseded workers make an intra-process collision reachable, and a zero-hole file still parses as a complete cached index — `src/session_index_io.cpp:211-214`
- [ ] (suggestion) The resident-tile cap exempts the coarsest level, which for a store with no overviews sidecar is the native level, so `resident_` grows unbounded while panning — `src/basemap_lod.cpp:563-605`
- [ ] (suggestion) The zoom blit scales a stale pixmap, painting the generalised coastline past its own fade floor for up to ~160 ms after the last wheel event — `src/sidescan_canvas.cpp:913-931`
- [ ] (suggestion) `openBag`, `runCubeLab`, `requestDrape` and `onTileSelectionChanged` lack the teardown guard `requestRender` has, and `openBag` re-arms `scan_cancel_` after `cancelWorkers()` may have set it — `src/sidescan_viewer_window.cpp:2862`
- [ ] (suggestion) (pre-existing, out of scope) A failed index scan reports as an empty normal completion, so a half-read bag is drawn, placed and summarised as a complete recording — `src/sidescan_viewer_window.cpp:2956,2992-3060`
- [ ] (suggestion) (governance) Middle-click is now compound and a left-click on a fix can reopen another recording; that refines the doc's stated rule — `docs/survey_explorer.md`
- [ ] (suggestion) (governance) Bag mode lists "middle-click seek" without the recentre, contradicting the correct description 15 lines later — `README.md:129`
- [ ] (suggestion) (governance) The header says the constants are seeded alike, but sigma_R here multiplies slant range while the real model uses depth — `src/sounding_uncertainty.hpp:71-78`
- [ ] (suggestion) (governance) `imagery/sidescan/tier1` is offered as a basemap layer, but Tier-1 is a columnar per-ping archive, not GGGS rasters — `src/world_layout.hpp:82`
