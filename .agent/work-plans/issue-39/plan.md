# Plan: Footprint-weighted drape resampling (fix the anti-aliasing gap)

## Issue

https://github.com/rolker/marine_perception_tools/issues/39 (Part of
[#36](https://github.com/rolker/marine_perception_tools/issues/36))

## Context

`drapePing()` (`src/sidescan_drape.cpp`) marches outward from nadir in
half-cell ground-distance steps, converts each step to a slant range, rounds
to the **nearest** raw amplitude sample, and stamps that one value uniformly
across an along-track strip (`half_width_m`, derived from ping-to-ping
spacing) with a straightness/range "quality score" deciding conflicts between
pings. Two effects compound to blur the georeferenced drape relative to the
waterfall it comes from, confirmed on a real Massabesic GCV bag
(`bizzyboat_sonar`, 2026-06-25, starboard channel):

- **Across-track**: 1.71 cm sample spacing vs. a 0.1 m default CUBE cell means
  ~6 raw samples land in one cell and 5 are discarded outright — the survivor
  is whichever the march happened to step on, not a mean.
- **Along-track**: the physical footprint (0.44° tx beamwidth × range) is
  3.8–26.7 cm over the swath, i.e. it grows past the 6.9 cm ping spacing
  beyond ~9 m range — so at far range, adjacent pings' footprints overlap and
  today's uniform-stamp-per-ping treatment doesn't reflect that.

`extend_surface_for_drape()` grows the CUBE surface to the sidescan swath and
membrane-fills gaps, but the drape lattice it produces always inherits the
CUBE surface's `cell_m` — so even a perfect kernel is bandlimited by whatever
cell size the operator picked for bathymetric gridding, which has nothing to
do with sidescan resolution.

**Resolved conventions this plan takes as given** (do not re-litigate):

- `tx_beamwidths`/`rx_beamwidths` on `marine_acoustic_msgs/PingInfo` are the
  **full** -3 dB width (halve for a half-extent) — corroborated by
  `apl-ocean-engineering/sonar_image_proc`'s
  `sonar_msg_metadata.py::min_elevation = -0.5*beamwidth` and
  [rviz_sonar_image#8](https://github.com/rolker/rviz_sonar_image/issues/8).
- "Not reported" appears in the wild as **either** an empty array (per
  `PingInfo.msg`'s own comment) **or** an all-zeros array of full length —
  the Norbit driver (`uri-ocean-robotics/norbit`,
  `norbit/src/conversions.cpp`) `.resize()`s without populating and carries
  an open upstream TODO about this ambiguity. Both must be treated as
  not-reported, plus any non-finite or non-positive value.
- Beamwidth is not angle-invariant for every array geometry
  ([marine_msgs#36](https://github.com/rolker/marine_msgs/issues/36), open) —
  acceptable to treat as constant for the Garmin GCV's single fixed
  along-track beam, but record it as a stated kernel limitation, not an
  unexamined assumption.

**Scope**: issue-suggested steps 1–2 only (footprint-weighted accumulation in
`drapePing()`; decouple the drape grid cell from the CUBE surface cell).
Step 3 (on-demand rasterization at view resolution) is explicitly deferred —
not planned here.

## Approach

1. **Add a beamwidth field to the ping geometry, sourced and gated at the bag
   boundary.**
   - Add `double tx_beamwidth_rad = 0.0;  // 0 == not reported` to
     `PingGeometry` (`src/sidescan_geometry.hpp`).
   - Add a small, bag-free, unit-testable resolver —
     `double resolve_reported_beamwidth(const std::vector<float> & beamwidths)`
     in `sidescan_geometry.{hpp,cpp}` — that returns `0.0` (not-reported) for:
     empty, size-mismatched-for-beam-0 (defensive), all values `<= 0`, or any
     non-finite value; otherwise returns `beamwidths[0]` (single fixed
     along-track beam — GCV is not a multi-beam array on this axis, matching
     ground truth #4's stated limitation). This isolates the "empty vs.
     all-zeros vs. garbage" ambiguity from #issue-39 ground truth 3 in one
     place instead of duplicating the check at every call site.
   - In `sidescan_bag_session.cpp` (~line 458, alongside the existing
     `sample0`/`metres_per_sample`/`lateral_sign` assignment), set
     `ping.geometry.tx_beamwidth_rad = resolve_reported_beamwidth(img.ping_info.tx_beamwidths);`.

2. **Fallback when beamwidth is not reported: degrade to today's behaviour,
   not a guessed constant.** When `tx_beamwidth_rad <= 0.0`, `drapePing()`
   keeps painting with the current *flat, unweighted* stamp across
   `half_width_m` (ping-spacing-derived) exactly as it does today. This is a
   deliberate no-regression fallback: a sensor that never reports beamwidth
   (or reports it as all-zeros per ground truth 3) gets the pre-#39 behaviour
   instead of a physically-invented footprint width.

3. **Along-track kernel: weighted accumulation replacing per-offset
   winner-take-all.** Today, every offset in `half_width_m` independently
   competes for its cell via `ping_score * range_score` (a given ping either
   fully owns a cell or not at all). Replace the per-offset flat stamp with:
   - Half-width for the *kernel*, when `tx_beamwidth_rad > 0`:
     `0.5 * tx_beamwidth_rad * slant` (full width → half-extent per the
     resolved convention; `slant`, not `t`, since beamwidth is defined against
     the acoustic path length).
   - A triangular (linear falloff to 0 at the half-width) weight per offset —
     simplest kernel that is zero at its support boundary (no discontinuity)
     and cheap; document as the deliberate initial choice, Gaussian is a
     possible follow-up, not blocking.
   - Per ping, accumulate `(weight, weight * amplitude)` into a per-cell
     scratch buffer scoped to this ping's footprint (not the whole grid).
     After the march completes, resolve each touched cell to
     `weighted_amplitude = Σ(weight·amplitude) / Σ(weight)` — the ping's
     candidate value for that cell.
   - Feed that resolved candidate into the **existing** unchanged
     conflict-resolution path: `score = ping_score * range_score` still
     decides whether this ping's candidate replaces `out.amplitude[cell]`,
     exactly as today. This confines the change to "what value does one ping
     offer a cell", leaving the composite/shadow logic alone.
   - When `tx_beamwidth_rad <= 0.0` (fallback), skip the weighted-average step
     and keep today's per-offset flat stamp + per-offset score compare
     unchanged — i.e. the new code path is additive, gated on beamwidth being
     reported.

4. **Across-track kernel: box-average the raw samples a march step
   represents, instead of nearest-sample.** At each march step `t` (ground
   distance, step = `0.5 * surface.cell_m` as today — see step 5 for why this
   stays tied to the *drape* cell, not the CUBE cell), compute the raw-sample
   index range covered by `[t - step/2, t + step/2]` via the existing
   slant/ground relation (`slant = sqrt(t² + dz²)`, converted through
   `metres_per_sample`/`sample0`, holding `dz` fixed across the small step —
   consistent with the existing shadow march's per-step terrain lookup).
   Average the in-range, finite amplitudes for that step's candidate value
   instead of rounding to one nearest sample. This is the direct fix for the
   "six samples in, one survives" loss described in the issue, and needs no
   beamwidth data — it applies unconditionally (across-track sample spacing is
   always known from `metres_per_sample`).

5. **Decouple the drape grid cell from the CUBE surface cell.**
   `extend_surface_for_drape()` currently copies `surface.cell_m` verbatim
   (`ext.cell_m = surface.cell_m;`, `sidescan_drape.cpp:301`). Add a
   `double drape_cell_m` parameter (independent of, and typically finer than,
   `surface.cell_m`). Before the existing grow/copy logic, **resample** the
   input CUBE surface onto the new cell size:
   - Bilinear-interpolate `depth` at each new-lattice node from its 4
     enclosing CUBE cells; a node is left `NaN` (unmeasured) unless **all
     four** neighbours are finite — no partial/blended "measured" claim from a
     genuinely-unmeasured neighbour. This reuses the *existing* multi-source
     BFS + Jacobi-relaxation membrane fill unchanged: it already treats `NaN`
     as "needs filling", so resampled-but-unmeasured nodes flow into the same
     path that already exists for the sidescan-outreach case.
   - `uncertainty`/`intensity` follow the same rule (NaN unless all 4
     neighbours finite); they are display-only on the drape path already.
   - The grow-to-swath and grid-guard (`max_nodes`) logic that follows
     operates on the resampled lattice's node count, so the existing
     `max_nodes` guard continues to bound total nodes regardless of
     `drape_cell_m` — a fine `drape_cell_m` reaches the guard sooner, which is
     the correct operator-guard behaviour (surfaced via the existing `note`
     mechanism, unchanged).
   - `drapePing()`/`drape_pass()` need no change to consume the finer lattice:
     they already read `surface.cell_m` for the march step and
     `half_width_m` floor, so they automatically operate at the finer
     resolution once handed the resampled `CubeSurface`.
   - Downstream: `sidescan_viewer_window.cpp` already renders the drape mesh
     from `cube_drape_terrain_` (the `extend_surface_for_drape()` output) when
     present, sized to match `cube_drape_.amplitude`
     (`sidescan_viewer_window.cpp:1133`, `:1336`) — so no mesh-building change
     is needed, only threading a `drape_cell_m` value through the call at
     `sidescan_viewer_window.cpp:1545`. Add a `drape_cell_spin_` control next
     to the existing `cube_cell_spin_` (pattern at `:1046`), defaulting to a
     value near the measured across-track sample spacing rounded up (e.g.
     0.02 m) — small enough that the sample-spacing/beamwidth kernel is the
     bandlimit, not the grid, without hardcoding sensor-specific numbers into
     the drape code itself.

6. **Keep the mirrored ping-level gate in sync (per the Issue Review action
   item).** `extend_surface_for_drape()`'s pre-filter
   (`sidescan_drape.cpp:239-243`) duplicates `drapePing()`'s ping-level skip
   condition by comment-documented intent (`fe8b2d7` fixed a prior
   desync that produced a false acoustic shadow past the swath edge). This
   plan does not change either gate's *condition* — `tx_beamwidth_rad` only
   affects the *kernel*, not whether a ping is skipped — but both call sites
   must be re-diffed at implementation time to confirm they stayed in sync,
   and a comment cross-reference should be added at both sites pointing at
   each other (neither currently says "keep this in sync with the other
   function").

## Files to Change

| File | Change |
|------|--------|
| `src/sidescan_geometry.hpp` | Add `tx_beamwidth_rad` to `PingGeometry`; declare `resolve_reported_beamwidth()`. |
| `src/sidescan_geometry.cpp` | Implement `resolve_reported_beamwidth()` (empty / all-zero / non-finite / non-positive → 0.0). |
| `src/sidescan_bag_session.cpp` | Populate `ping.geometry.tx_beamwidth_rad` from `img.ping_info.tx_beamwidths` via the resolver. |
| `src/sidescan_drape.hpp` | Add `drape_cell_m` parameter to `extend_surface_for_drape()`; document the new kernel behaviour in the file-level comment block. |
| `src/sidescan_drape.cpp` | Weighted along-track accumulation + box-averaged across-track sampling in `drapePing()`; resample-before-grow in `extend_surface_for_drape()`; sync-gate cross-reference comments. |
| `src/sidescan_viewer_window.hpp`/`.cpp` | Add `drape_cell_spin_` control; thread its value into the `extend_surface_for_drape()` call (~line 1545). |
| `test/test_sidescan_drape.cpp` | New cases (see below). |
| `test/test_sidescan_geometry.cpp` | New cases for `resolve_reported_beamwidth()` (empty, all-zero, non-finite, valid). |
| `.agents/README.md` | Update the drape description (currently documents "nearest sample, no averaging") once the kernel lands — see Documentation Impact. |

### Test plan (in `test_sidescan_drape.cpp`, extending the existing synthetic-terrain/synthetic-ping harness)

- **Beamwidth convention pinned**: single bright synthetic sample at known
  range `R` with `tx_beamwidth_rad = θ`; assert the painted along-track
  extent matches `0.5·θ·R` (half-extent from full-width), not `θ·R` — this is
  the regression guard for the resolved rviz_sonar_image#8 convention and is
  explicitly called out as required in the Issue Review.
- **Not-reported fallback — three shapes**: empty `tx_beamwidths`,
  all-zeros `tx_beamwidths` (Norbit-shape), and a single non-finite entry —
  all three must produce identical output to today's flat-stamp behaviour
  (byte-for-byte or tolerance-equal amplitude/shadow arrays vs. a
  `tx_beamwidth_rad = 0.0` control run).
- **Across-track anti-aliasing**: synthetic ping with a narrow
  (sub-cell-width) bright target in the raw sample array; assert the drape
  preserves the target's amplitude and position instead of aliasing it away
  or having it be silently dropped by nearest-sample rounding (the issue's
  stated acceptance criterion).
- **Grid decoupling**: build a coarse `CubeSurface`, call
  `extend_surface_for_drape()` with a finer `drape_cell_m`, assert the
  returned surface's `nx`/`ny`/`cell_m` reflect the finer lattice and that
  previously-measured coarse cells remain finite at every resampled node
  that falls fully inside them; assert a node whose 4 coarse neighbours
  include an unmeasured (`NaN`) one is itself `NaN` before the membrane fill
  runs (i.e. resampling doesn't fabricate "measured" data).
- **`max_nodes` guard still bounds the finer lattice**: existing guard test
  pattern, re-run with a `drape_cell_m` smaller than `surface.cell_m`,
  asserting the guard note still fires and node count stays `<= max_nodes`.
- **Gate sync regression**: a ping that `drapePing()` skips (e.g.
  `altitude <= 0`) must also be excluded from `extend_surface_for_drape()`'s
  bounds computation — extend the existing coverage (if present) or add it;
  this is the direct regression test for the `fe8b2d7` bug class named in the
  Issue Review action item.

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Documentation accuracy / verify against source | All quantities in this plan (beamwidth field types, empty/all-zero shapes, msg definitions) were verified against `PingInfo.msg`/`RawSonarImage.msg` in `/opt/ros/jazzy/share/marine_acoustic_msgs`, not assumed. |
| Quality standard — fix bugs completely | Both halves of the blur (across-track discard, along-track uniform stamp) are addressed together rather than picking one; the not-reported fallback is explicit rather than silently producing a wrong footprint. |
| Never leave "good enough" when the proper fix is in reach | Considered doing only the across-track box-average (cheaper) and leaving along-track untouched; rejected — the issue's own measurements show the along-track footprint is the *larger* source of anisotropy (16:1 aspect at swath edge), so a partial fix would leave the dominant blur source unaddressed. |
| Test-driven, not "trust the math" | Every new numeric behaviour (convention, fallback, decoupled grid) gets a dedicated synthetic test rather than relying on the formula being self-evidently right. |

## ADR Compliance

No ADR in `docs/decisions/` in this project repo governs sonar geometry or
resampling kernels; none triggered. (Workspace-level ADRs on CI/worktree
process are unaffected — this is a pure algorithm change inside an existing
library.)

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `PingGeometry` (new field) | Any other construction site of `PingGeometry` (test harnesses in `test_sidescan_drape.cpp`, `test_sidescan_geometry.cpp`) | Yes — new tests construct it explicitly; existing tests default-construct so `tx_beamwidth_rad = 0.0` (not-reported) preserves their current pass/fail behaviour with no edits needed. |
| `extend_surface_for_drape()` signature (`drape_cell_m` added) | The one call site (`sidescan_viewer_window.cpp:1545`) and every direct test call in `test_sidescan_drape.cpp` | Yes — both listed in Files to Change. |
| Drape kernel behaviour | `.agents/README.md`'s drape description ("nearest sample, no averaging") | Yes — see Documentation Impact below. |
| `SidescanDrape`/`CubeSurface` node counts no longer equal `cube_surface_`'s | Any other `cube_surface_.cell_m`-keyed UI text (status line at `sidescan_viewer_window.cpp:939` reports `cube_surface_.cell_m`, which stays correct — that status line describes the *CUBE* grid, not the drape grid; no change needed there, but worth a fresh status-line mention of the drape cell size once `drape_cell_spin_` exists) | Partially — adding a drape-cell-size readout alongside the new spinbox is a UI-polish item folded into the `drape_cell_spin_` work, not a separate file; no additional file listed. |

## Documentation & Instruction Impact

- **Stale docs** (must land in this PR): `.agents/README.md`'s
  `marine_perception_tools` inventory row and/or `sidescan_drape.hpp`'s
  file-level comment currently state "Nearest sample per cell, no
  averaging" — both must be updated to describe the footprint-weighted
  kernel once it lands, per the Issue Review's action item to update
  `.agents/README.md`. `sidescan_drape.hpp`'s header comment block is
  effectively part of this same stale-docs item since it's the primary
  design-intent doc for this file.
- **Agent-instruction candidates** (proposals only): the "empty vs.
  all-zeros" not-reported ambiguity (ground truth 3, Norbit driver) is a
  reusable gotcha for any future consumer of `PingInfo`/sonar metadata
  fields in this workspace — worth a one-line entry in
  `.agent/knowledge/` (workspace-level, not this repo) the next time
  someone touches sonar metadata parsing. Not proposed as part of this PR;
  flagged for the operator to decide separately, since it's workspace-level
  and this PR is scoped to the project repo.

## Open Questions

- [ ] Triangular vs. Gaussian along-track kernel shape (step 3) — plan
  chooses triangular for simplicity/cost; confirm this is acceptable or if
  a Gaussian is preferred before implementation, since it changes the exact
  half-width test assertion.
- [ ] Default value for the new `drape_cell_spin_` UI control (step 5) —
  plan suggests ~0.02 m (near the measured GCV across-track sample
  spacing) but this is sensor-specific; confirm whether a fixed default is
  acceptable or whether it should default to `cube_cell_spin_`'s current
  value (i.e. opt-in decoupling) to avoid surprising existing users with a
  much larger grid on first use after this ships.

## Estimated Scope

Single PR. The change is confined to `sidescan_drape.{hpp,cpp}`,
`sidescan_geometry.{hpp,cpp}`, one call site in `sidescan_bag_session.cpp`,
one call site + one new UI control in `sidescan_viewer_window.{hpp,cpp}`, and
their tests plus the `.agents/README.md` stale-docs update. No new files,
no cross-repo changes.
