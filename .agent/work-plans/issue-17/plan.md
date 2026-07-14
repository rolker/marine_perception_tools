# Plan: Interval loader — open sidescan_target_viewer cued to a survey-index pass

## Issue

https://github.com/rolker/marine_perception_tools/issues/17

## Context

`sidescan_target_viewer` currently accepts only a bag directory as its first positional
argument and opens positioned at distance 0. To review a target from `survey_index_query
--json`, the user must open the bag and scrub manually to the right minute of a multi-GB
recording. This issue adds `--start`/`--end` CLI flags (nanoseconds or ISO 8601) that cue
the scrub to the pass interval automatically.

`SidescanBagSession::buildIndex()` already reads the whole bag in a single lightweight
metadata pass (TF cache + ping header timestamps + boat-pose table, no sample data). Sample
data is only read by `readWindow()` for the current scrub window. The interval loader cues
the VIEW, not the index: it converts the requested timestamps to an along-track distance
after the index is built, then seeks the scrub slider there. The full metadata pass is
retained for TF continuity — this is documented in the PR and code (see Architectural
Decision below).

## Approach

1. **Add `distance_interval()` free function** — in `sidescan_bag_session.hpp`, a header-
   visible free function `distance_interval(const SessionIndex&, int64_t t_start_ns, int64_t
   t_end_ns) -> std::pair<double, double>` does a binary search on
   `SessionIndex::pings` (sorted by `stamp_ns`) to find the along-track distances
   bracketing the requested time window. Returns `{0.0, total_distance_m}` when either
   bound is zero (= no cue). Qt-free, bag-I/O-free, testable independently.

2. **Add interval unit test** — `test/test_sidescan_interval.cpp`: construct a synthetic
   `SessionIndex` with hand-crafted pings (no bag), assert `distance_interval()` returns
   correct `{dist_lo, dist_hi}` for in-range, clamped, and zero-bound inputs. Register in
   `CMakeLists.txt` as `test_sidescan_interval` linking `sidescan_core`.

3. **Extend `openBag()` signature** — `SidescanViewerWindow::openBag(bag_uri, t_start_ns,
   t_end_ns)` (default 0 for both = no cue). Store `pending_cue_start_ns_` /
   `pending_cue_end_ns_` as members; clear them after applying.

4. **Apply cue after index completes** — in `onIndexProgress()` when `done == true` and
   `pending_cue_start_ns_ != 0`, call `distance_interval()` on the snapshot and set
   `scrub_->setValue(static_cast<int>(dist_lo))` (scrub_ units are metres). Log the
   resolved window to the status label.

5. **Add `QCommandLineParser` to `sidescan_viewer_main.cpp`** — parse `--start <ns|ISO>`
   and `--end <ns|ISO>`. Parse order: try `toLongLong()` for nanosecond integers first;
   fall back to `QDateTime::fromString(s, Qt::ISODateWithMs)` and convert to ns. Validate
   that if either flag is provided, both are required. Pass parsed ns values to `openBag()`.

6. **Update `.agents/README.md`** — add the new CLI surface to the package inventory row
   and note the full-bag metadata scan vs. sample-data-only window read.

## Architectural Decision: Full-bag metadata scan is retained

`buildIndex()` reads the whole bag once for TF and boat-pose data; this is necessary for
correct cumulative-distance assignment across the whole recording. The interval loader
does NOT restrict the index build — it cues the scrub AFTER the index is complete. Sample
data (ping amplitudes) is only read by `readWindow()` for the active scrub window, so the
user never waits for a whole-bag sample read. This limitation is documented in the PR
description and in a code comment on `openBag()`.

## Files to Change

| File | Change |
|------|--------|
| `src/sidescan_bag_session.hpp` | Add `distance_interval()` free function declaration after `SessionIndex` struct |
| `src/sidescan_bag_session.cpp` | Implement `distance_interval()` via `std::lower_bound` on `pings` by `stamp_ns` |
| `src/sidescan_viewer_window.hpp` | Extend `openBag()` signature; add `pending_cue_start_ns_`/`pending_cue_end_ns_` members |
| `src/sidescan_viewer_window.cpp` | Store cue params; apply in `onIndexProgress(done=true)` |
| `src/sidescan_viewer_main.cpp` | Replace raw `argv[1]` with `QCommandLineParser`; parse `--start`/`--end` |
| `test/test_sidescan_interval.cpp` | New GTest for `distance_interval()` — synthetic pings, no bag |
| `CMakeLists.txt` | Add `ament_add_gtest(test_sidescan_interval ...)` linking `sidescan_core` |
| `.agents/README.md` | Update CLI surface + add full-bag metadata scan pitfall note |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | `QCommandLineParser` + one free function + scrub cue — no new UI panes, no new deps. Non-goals (overview map, multi-pass) deferred per issue. |
| Improve incrementally | Single PR; does not touch the indexing path or buffer policy. |
| Test what breaks | Interval-selection logic (binary search on stamp_ns) is the only new logic at risk; covered by a bag-I/O-free test. |
| A change includes its consequences | CLI surface in `.agents/README.md` updated in same PR; architectural choice documented in PR and code. |
| Capture decisions, not just implementations | Full-bag metadata scan rationale documented in plan, PR description, and code comment. |
| Human control and transparency | `--start`/`--end` are explicit, scriptable flags; no hidden automation. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0008 — ROS 2 Conventions | Yes | New C++ follows Apache 2.0 header, `snake_case` symbols, existing project style. |
| ADR-0001 — Adopt ADRs | Watch | Full-bag scan decision documented in PR description and code comment; spans only this package so a new ADR is not warranted. |
| ADR-0002 — Worktree isolation | OK | Feature branch `feature/issue-17` already active. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `openBag()` signature | `sidescan_viewer_main.cpp` call site | Yes |
| CLI arguments added | `.agents/README.md` package inventory | Yes |
| New free function in `sidescan_bag_session.hpp` | No downstream consumers outside this package | N/A |

## Decisions (plan review, 2026-07-14)

- **Cue target (review must-fix)**: the scrub head paints the TRAILING window
  `[head − window_len, head]`, so cueing `head = dist_lo` would show the track
  *before* the pass. Cue `head = min(dist_lo + window_len_m_, dist_hi)`: a pass
  shorter than the window is fully in view (with leading context); a longer pass
  opens on its first window-length.
- **`distance_interval()` returns `std::optional<std::pair<double,double>>`**
  (refines the planned bare pair): `nullopt` when no *posed* ping falls in the
  window (outside the bag's time range / nothing resolved) — the caller reports
  "cue window outside bag" instead of silently cueing to 0. Zero bounds = no cue,
  handled by the caller before calling.
- **ISO-8601 parse forces UTC (review suggestion)**: `QDateTime::fromString(s,
  Qt::ISODateWithMs)`; invalid → clear CLI error; a string without an offset
  parses as LocalTime → reinterpreted as UTC via `setTimeSpec(Qt::UTC)`.
- **Flags stay `--start`/`--end`** (suffix-free, unlike sea_surface_tuner's
  `--start-s` seconds flags); the ns-or-ISO meaning is explicit in `--help`.
- **Cue applies at the END of the `done` block** in `onIndexProgress()` (after
  the range is finalized) and `pending_cue_*` are cleared there.
- **`.agents/README.md` gets a viewer entry** (the inventory currently describes
  only sea_surface_tuner), not an append to the tuner row.

## Open Questions

- None. The acceptance criterion allows documenting the full-bag metadata scan rather
  than requiring a seek-based alternative; the plan takes this path.

## Estimated Scope

Single PR.
