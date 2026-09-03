# Plan: Texture-mapped sidescan drape (replaces per-cell resampling with GPU blending)

## Issue

https://github.com/rolker/marine_perception_tools/issues/39 (Part of
[#36](https://github.com/rolker/marine_perception_tools/issues/36))

## Revision note (2026-09-03)

This is a full revision of the prior plan (`72116fb`), which got
`changes-requested` (`## Plan Review`, `fd70112`). The review's must-fix
(Finding 1) is fatal to the previous approach: `drapePing()` produces exactly
**one** amplitude per march step and stamps it identically across every
along-track offset in that step's strip — a weighted mean over those offsets
algebraically collapses to that same constant, because there is no
per-offset amplitude data within a single ping to weight between. Real
along-track resolution beyond one ping's spacing only exists **between**
pings, and the previous plan's step 3 explicitly fed its (inert) result into
the existing per-cell winner-take-all conflict path, which precluded
blending across pings by construction.

The operator has redirected this plan around **texture mapping**: build each
pass's imagery as a texture in the sonar's own native (ping x sample)
geometry, and let the mesh vertices carry texture coordinates (UVs) computed
from the same slant-range projection `drapePing()` already does — GPU
bilinear filtering then interpolates *between* ping rows at render time,
which is the real, non-degenerate cross-ping blend the review said was
missing. This also **subsumes the issue's deferred step 3** (on-demand
rasterization at view resolution): because imagery is never resampled onto a
grid, the georeferenced view's sharpness is bounded by the texture, not by
any world-space cell size, matching the waterfall's 1:1 fidelity while
keeping the corrected (occlusion-aware) geometry. That step is therefore
folded into this plan rather than deferred.

**Everything below replaces the previous plan's Approach (steps 1-6, 174-285
old numbering) except where explicitly marked "carried over unchanged."**

## Context (verified against source, 2026-09-03)

- `drapePing()` (`src/sidescan_drape.cpp:78-211`) marches outward from
  nadir in half-CUBE-cell ground steps, rounds each step to the nearest raw
  amplitude sample, and stamps it across an along-track strip
  (`half_width_m`, ping-spacing-derived) with `score = ping_score *
  range_score` deciding conflicts against `out.painted_score[cell]`. This
  is the mechanism the review's Finding 1 showed cannot be made to blend
  along-track without changing what "one cell's value" means.
- `extend_surface_for_drape()` (`sidescan_drape.cpp:215-394`) grows the CUBE
  surface to the swath and membrane-fills gaps; it mirrors `drapePing()`'s
  ping-level skip gate by comment-documented intent
  (`sidescan_drape.cpp:232-238`, referencing the `fe8b2d7` false-shadow
  fix). This function's *terrain* role (the relief the imagery lands on)
  is unchanged by this plan — the mesh still comes from here, at CUBE/bathy
  resolution.
- `PointCloudView::setSurface(positions_xyz, colors_rgb, indices)`
  (`src/point_cloud_view.hpp:115-117`) takes **per-vertex RGB**. This is the
  hard ceiling on today's imagery resolution: a vertex (one per CUBE node,
  `src/cube_lab.cpp` `build_cube_mesh_colored`) can carry only one colour,
  so no rendering change downstream of `SidescanDrape` can exceed CUBE-cell
  resolution. `upload_surface()` (`point_cloud_view.cpp:480-511`) uploads
  position (location 0) and colour (location 1) into the **same** shader
  program (`kVertexShader`/`kFragmentShader`, `point_cloud_view.cpp:46-67`)
  used for the point cloud itself — there is currently no texture sampling
  anywhere in this file (`grep` confirms no `QOpenGLTexture`/`glGenTextures`
  in `src/`).
- The sidescan-shade path in `sidescan_viewer_window.cpp:1126-1206` builds
  `node_rgb` (one RGB triple per terrain node) on the **CPU**, from
  `cube_drape_.amplitude`/`.shadow` plus a baked `marine_colormap` LUT, then
  calls `build_cube_mesh_colored()`. This CPU colour-mapping step moves to
  the GPU under this plan (see Approach A.4).
- The composite path (`requestDrape()`, `sidescan_viewer_window.cpp:1487-1560`)
  currently composites by **concatenating** every target pass's `WindowPing`s
  into one list and calling `drape_pass()` once — conflict resolution is the
  same per-cell winner-take-all `score` compare used for a single pass. This
  plan replaces that with the GPU depth-score compositing in Approach B, but
  the *scoring formula itself* (`ping_score * range_score`, straightness x
  range-closeness, both `RangeScoreMode` variants) is carried over unchanged
  in meaning — see B.2.
- `PingGeometry` (`src/sidescan_geometry.hpp:91-100`) has no beamwidth
  field. `sidescan_geometry.hpp` is **header-only** (no `.cpp` — confirmed:
  `src/sidescan_geometry.cpp` does not exist; the previous plan's Files-to-
  Change table was wrong to list one, per Plan Review Finding 3's own
  standard of checking claims against source).
- `test/test_point_cloud_view.cpp` already has the precedent for what this
  plan needs: `gl_available()` self-skip guard, `QOffscreenSurface` +
  `QOpenGLContext` setup, `grabFramebuffer()` + pixel-colour readback
  assertions (`RendersAndClears`, `MultiPassColour` — the latter already
  asserts *which* of two colour families dominates which screen region,
  the same shape of assertion this plan's composite tests need). Wired via
  `CMakeLists.txt` `set_tests_properties(test_point_cloud_view PROPERTIES
  ENVIRONMENT "QT_QPA_PLATFORM=offscreen;LIBGL_ALWAYS_SOFTWARE=1")` — this
  is the **only** test target with that environment today; a new texture/
  composite test target needs the same property added.
- `test/test_sidescan_drape.cpp` has `StraightPassBeatsTurningPassInComposite`
  and `RangeScoreModeFlipsConflicts` (lines 245-296) — pure-CPU tests of
  today's per-cell winner-take-all. Both must be ported, not dropped (see
  Approach C).
- Issue [#34](https://github.com/rolker/marine_perception_tools/issues/34)
  (open): `setSurface()` does no buffer/index validation. This plan adds a
  **new** entry point (`setSurfaceTextured`, Approach A.4) with the same
  validation posture as today's `setSurface()` — i.e. it does not fix #34,
  and must not introduce a *new* way to feed it mismatched buffers that
  #34's eventual fix wouldn't also cover (same failure shape: trust the
  caller, `.size()`-mismatch guarded by early-return, not asserted).

**Resolved conventions this plan takes as given** (unchanged from the prior
plan, still correct — do not re-litigate):

- `tx_beamwidths`/`rx_beamwidths` on `marine_acoustic_msgs/PingInfo` are the
  **full** -3 dB width (halve for a half-extent) — corroborated by
  `apl-ocean-engineering/sonar_image_proc`'s `sonar_msg_metadata.py::
  min_elevation = -0.5*beamwidth` and
  [rviz_sonar_image#8](https://github.com/rolker/rviz_sonar_image/issues/8).
- "Not reported" appears as **either** an empty array **or** an all-zeros
  array of full length (Norbit driver shape) — both, plus any non-finite or
  non-positive value, are treated as not-reported.
- Beamwidth is not angle-invariant for every array geometry
  ([marine_msgs#36](https://github.com/rolker/marine_msgs/issues/36),
  open); acceptable to treat as constant for the Garmin GCV's single fixed
  along-track beam — record as a stated kernel limitation.
- Measured Garmin GCV ground truth (frequency, sample spacing, beamwidths,
  ping interval, footprint-vs-range table, the ~9 m regime change) stands as
  given in the issue and is not re-derived here.
- A 200 m pass is roughly 2000 x 2900 texels at native-ish resolution
  (~6 MB single-channel) — texture memory is not the constraint; the
  driver-reported `GL_MAX_TEXTURE_SIZE` (commonly 16384, but **must be
  queried at runtime, never hardcoded**) is, for passes long enough to
  exceed it in the row dimension.

**Scope**: build the single-pass texture pipeline (A) and the composite
GPU-compositing pipeline (B) in one PR — they share the per-pass texture
builder and the mesh/UV plumbing, so splitting them would leave the
single-pass half unable to demonstrate the conflict-resolution semantics
its own tests need to port. Testability risk and mitigation (C) is planned
alongside, not deferred.

## Approach

### A. Single pass — decouple imagery from mesh resolution

**A.1. Beamwidth plumbing — carried over from the prior plan, with a new
   plausibility-range guard added mid-plan (operator-flagged, 2026-09-03).**
   - Add `double tx_beamwidth_rad = 0.0;  // 0 == not reported` to
     `PingGeometry` (`src/sidescan_geometry.hpp`).
   - Add `inline double resolve_reported_beamwidth(const std::vector<float>
     & beamwidths)` **in `sidescan_geometry.hpp` itself** (the file is
     header-only; there is no `.cpp` to put it in) — returns `0.0` for
     empty, all-values-`<=0`, any non-finite entry, **or a value outside a
     plausible radians range**; otherwise returns `beamwidths[0]` (single
     fixed along-track beam).
   - **Plausibility range, and why it's needed.** This field has now failed
     three distinct, independent ways across this ecosystem, all traceable
     to `PingInfo.msg` underspecifying it:
     1. full-width vs. half-extent
        ([rviz_sonar_image#8](https://github.com/rolker/rviz_sonar_image/issues/8) —
        a fan rendered 2x too wide).
     2. empty vs. all-zeros for "not reported" (the Norbit driver,
        `uri-ocean-robotics/norbit`, `.resize()`s without populating, with
        an open upstream TODO).
     3. **degrees vs. radians** —
        `cube_bathymetry/src/error_model.cpp:236-238` multiplies
        `rx_beamwidths[i]` by `M_PI/180.0`, i.e. treats the field as
        **degrees**, while `PingInfo.msg` documents radians. The M3 driver
        (`marine_tools/kongsberg_em_bridge/kongsberg_em_bridge/node.py:
        547-550`) deliberately leaves both arrays **empty** specifically to
        dodge this mismatch, with a comment saying so — i.e. a real,
        currently-shipping producer treats the message's own unit
        convention as unsafe to populate. (The driver's comment cites
        "cube_bathymetry#30" as tracking this — that issue is **closed**
        and its actual subject is validating the error model against
        Calder's original formulation, not the deg/rad mismatch; the units
        bug is currently **untracked**, and this plan does not claim
        otherwise.)

        A degrees-valued array read as radians arrives ~57x too large. For
        this resolver, "not reported" and "implausible" must collapse to
        the same safe fallback — trusting an out-of-range value would
        silently scale the along-track footprint by ~57x instead of
        falling back to today's ping-spacing-derived behaviour. **The
        fallback path is therefore not merely a nicety for missing data —
        it is the guard that makes a unit-confused producer fail safe.**
     - Bounds: reject (treat as not-reported) any value `<= 0` (kept from
       before) or `> 1.2` rad. Justification: the widest legitimate
       beamwidth in this issue's own evidence is the Garmin GCV's
       **across-track** rx beamwidth, 55 deg = 0.960 rad (used here as a
       ceiling reference even though `tx_beamwidth_rad` is the along-track
       figure, since it is the largest plausible acoustic beam angle in
       hand); the along-track tx value actually consumed is 0.44 deg =
       0.00768 rad, two orders of magnitude smaller. `1.2` rad (~69 deg)
       sits comfortably above the widest legitimate value seen (0.96 rad)
       with headroom for a wider-but-still-physical beam, while a
       degrees-mistaken value in the sidescan along-track range (single
       digits to low tens of degrees numerically, read as "radians") lands
       at 1.5-30+ rad — solidly caught. A value from a genuinely
       wide-beam sensor between 0.96 and 1.2 rad is judged less likely and
       less costly (a slightly-too-narrow along-track tile at worst) than
       admitting a degrees mix-up.
   - In `sidescan_bag_session.cpp` (~line 458, alongside the existing
     `sample0`/`metres_per_sample`/`lateral_sign` assignment), set
     `ping.geometry.tx_beamwidth_rad =
     resolve_reported_beamwidth(img.ping_info.tx_beamwidths);`.
   - **Where beamwidth is actually used now**: not as a per-offset kernel
     half-width (that mechanism is gone — see Revision note). It becomes
     the along-track extent of a texture ROW when building the per-ping
     mesh-UV mapping in A.3: a ping's row occupies a V-span derived from
     `0.5 * tx_beamwidth_rad * slant` where reported, falling back to
     today's ping-spacing-derived `half_width_m` (`drapePing()`'s existing
     formula, `sidescan_drape.cpp:73-75`/`414-431`) where not reported —
     i.e. the not-reported fallback degrades to *today's along-track
     tiling*, not a guessed footprint, same intent as the prior plan.

**A.2. Per-pass, per-channel-side texture builder.**
   - New file `src/sidescan_texture.hpp`/`.cpp` (new; `WindowPing` already
     separates Port/Starboard/Down as distinct entries — confirmed
     `SidescanChannel` in `sidescan_geometry.hpp:34-39` and `WindowPing`'s
     `channel` field, `sidescan_bag_session.hpp:87-92` — so grouping by
     channel is filtering the existing list, not a new concept).
   - `struct SidescanTexture { int rows = 0; int cols = 0; std::vector<float>
     amplitude; std::vector<std::uint8_t> shadow; std::vector<float>
     ping_score; /* size == rows, one per row */ std::vector<double>
     row_max_slant, row_sample0_slant; /* size == rows; per-row range-score
     inputs, since sample0/metres_per_sample are not guaranteed constant
     across a pass */ std::size_t pings_used = 0; std::size_t pings_skipped
     = 0; };`
   - Build one `SidescanTexture` per (pass, channel) — Down is never built
     (it never paints today either: `lateral_sign == 0` is part of the
     existing skip gate, carried over unchanged, A.5).
   - **Rows** = that channel's pings in time order, one row per ping — this
     is the axis GPU bilinear filtering will interpolate across, which is
     where the real, non-degenerate cross-ping blend happens (distinct
     rows hold each ping's own distinct amplitude data — nothing like
     Finding 1's constant-repeated-per-offset problem, because a row's
     neighbour is a *different* ping's actual samples, not the same ping's
     value copied sideways).
   - **Columns**: a chosen column pitch (`texture_col_pitch_m`, an
     across-track ground-distance step — plays the role the previous
     plan's `drape_cell_m` played, but now bounds the *texture*, not a
     world grid). For each column step `t` (ground distance from nadir),
     **box-average the raw samples the step covers** — this reuses the
     prior plan's step-4 math verbatim (slant/ground relation, holding
     `dz` fixed across the small step, consistent with the existing shadow
     march's per-step terrain lookup) and the prior plan's dilution-aware
     acceptance criterion (Plan Review Finding 2, folded into this plan's
     test plan directly rather than needing a second fix-up). This is
     **carried over from the prior plan**, but its output now determines
     one texel's value, not one world cell's value.
   - `ping_score[row]` and the per-row range-score inputs are computed
     once per ping exactly as `drape_pass()` computes `ping_score` today
     (`sidescan_drape.cpp:433-451`, straightness from yaw-rate against
     neighbours) — this logic **moves** from `drape_pass()` into the
     texture builder (still a pure function over a ping sequence, still
     unit-testable without GL) but is not changed in formula.
   - Shadow is per-texel, from the same first-return occlusion march
     `drapePing()` already does (`sidescan_drape.cpp:121-182`), but now
     walking `t` at the *column* pitch instead of half the CUBE cell — so
     shadow boundaries stay sharp at column (near-sample) resolution
     rather than being quantized to CUBE-cell resolution, which is the
     concrete fix for the operator's "shadows must stay sharp" requirement
     (per-vertex visibility, the alternative, would have quantized them to
     mesh resolution — explicitly rejected).
   - Non-uniform sample counts across a pass's pings (if the session's
     `metres_per_sample`/sample count ever changes mid-recording) pad
     shorter rows with an invalid/NaN sentinel rather than resampling —
     flagged as an **open question** (needs checking against a real bag
     with a mid-pass parameter change; not something the ground truth in
     this issue rules out or in).

**A.3. Mesh UV assignment — the mechanism that makes cross-ping blending real.**
   - The mesh is **unchanged**: still `extend_surface_for_drape()`'s output
     terrain, still triangulated by `build_cube_mesh`-family code at
     CUBE/bathy resolution. **Do not resample the surface for imagery** —
     this is the core of the operator's redirection and is what keeps this
     plan from reopening the old plan's grid-decoupling step (the prior
     plan's step 5, `extend_surface_for_drape()` + `drape_cell_m`, is
     **dropped entirely** — texture resolution replaces it).
   - New `CubeSurfaceMesh build_cube_mesh_uv(const CubeSurface & surface,
     CubeMeshStyle style)` in `src/cube_lab.hpp`/`.cpp`, a sibling of the
     existing `build_cube_mesh_colored()` (`cube_lab.hpp:173-176`): same
     triangulation, but each output vertex carries a placeholder UV slot
     instead of an RGB triple — the *positions* it produces are identical
     to what `build_cube_mesh_colored()` produces today (verified: same
     quad-corner-mean-height / per-node geometry for `CrispSmooth`, no
     geometry change).
   - UV computation per mesh vertex (world position on the terrain,
     already known from the mesh build): find the vertex's along-track arc
     position on the channel's ping-sensor polyline (nearest two
     consecutive same-channel pings bracketing the vertex, i.e. the same
     "nearest pings" neighbourhood `drapePing()`'s march already walks
     through, generalized from "nearest one" to "nearest bracketing pair
     for interpolation") — `V` = fractional row between them; `U` = column
     from the ground/slant range computed against the (nearer, or
     linearly-interpolated) ping's across-track ray, via the same
     box-averaged column mapping as A.2. This is genuinely continuous and
     non-degenerate: unlike the prior plan's step 3, `V` interpolates
     between **two different rows holding two different pings' real
     amplitude data** — there is no algebraic collapse to a repeated
     constant here, which is the direct answer to Plan Review Finding 1.
   - A vertex outside the pass's coverage (no bracketing pings within a
     reasonable along-track distance, or across-track beyond the swath)
     gets an out-of-range/sentinel UV; the fragment shader (A.4) treats an
     out-of-range UV as "unseen" (today's dim-grey state), matching the
     existing `measured`/`shadow`/`painted` tri-state.
   - This UV-assignment pass is genuinely per-pass — in single-pass mode it
     is the whole story; in composite mode (B) each pass gets its own UV
     assignment against its own mesh copy, and conflict resolution happens
     downstream in the GPU compositing step, not here.

**A.4. `PointCloudView` gains a textured-surface path.**
   - New method `void setSurfaceTexture(std::vector<float> positions_xyz,
     std::vector<float> uvs, std::vector<std::uint32_t> indices,
     const SidescanTextureGpu & tex)` (name/shape to be finalized at
     implementation; `SidescanTextureGpu` bundles the amplitude+shadow
     pixel data, `rows`/`cols`, and the colour-LUT parameters needed at
     draw time). This is **additive** — `setSurface()` (per-vertex-colour)
     stays exactly as-is for the CUBE `Depth`/`Uncertainty`/`Intensity`
     shade modes (`sidescan_viewer_window.cpp` shade_i != 3 path,
     `cube_lab.cpp` `build_cube_mesh`), which have no reason to move to
     textures.
   - New GL state: a second shader program (or the existing program
     extended with a `u_use_texture` uniform switch — decide at
     implementation, the two-program route is cleaner since the vertex
     attribute layout differs: position + UV instead of position + colour)
     that samples an amplitude texture (`GL_R32F` or `GL_R16F`, one
     channel) and a shadow/mask texture (`GL_R8UI` or packed into the
     amplitude texture's alpha if using RGBA — decide at implementation),
     plus a 1D colour-LUT texture (256x1 RGB) built once per palette/range
     change from the same `marine_colormap::bake_lut()` output that is
     uploaded today's CPU path bakes — **this moves the lo/hi normalize +
     LUT lookup from the CPU loop in `sidescan_viewer_window.cpp:1178-1195`
     into the fragment shader**, a real architectural shift worth calling
     out: the "unseen" (dim grey) / "shadow" (near-black) / "painted"
     (LUT colour) tri-state becomes a fragment-shader branch on the
     shadow-texture texel and UV validity, not a CPU per-node loop.
   - Texture filtering: `GL_LINEAR` for the amplitude texture (the
     bilinear cross-ping/cross-column blend this whole plan is built on);
     `GL_NEAREST` for the shadow/mask texture (a shadow boundary must stay
     a hard edge, not blur into "half-shadow" — blending mask values would
     visually contradict the "shadows must stay sharp" requirement).
   - Tiling for passes whose texture would exceed the runtime-queried
     `GL_MAX_TEXTURE_SIZE` in the row dimension: split into multiple
     row-contiguous tiles, each its own texture + UV row-offset, rendered
     as separate draw calls (or texture array layers, decide at
     implementation) — sized from the actual queried limit, never a
     hardcoded 16384.

**A.5. Ping-level gate — carried over unchanged from the prior plan.**
   The texture builder (A.2) and any UV-assignment code (A.3) that walks
   pings must apply the **identical** ping-level skip condition
   `drapePing()` uses (`metres_per_sample <= 0`, `lateral_sign == 0`,
   `altitude <= 0`, empty `amplitudes`) — this is new code, so it is a
   fresh place to reintroduce the `fe8b2d7` false-shadow-past-swath-edge
   bug class if the gate is copied loosely rather than shared. Add a single
   `inline bool ping_is_drapable(const PingGeometry & g, const WindowPing &
   p)` helper in `sidescan_geometry.hpp` that `drapePing()`,
   `extend_surface_for_drape()`, and the new texture builder all call, so
   there is exactly one place this condition lives instead of three
   independently-maintained copies (an improvement over today's two-copy
   "mirror by comment" arrangement, and the direct fix for the Issue
   Review's gate-sync action item).

### B. Composite (multiple passes) — GPU depth-score compositing

**B.1. Offscreen ground-space render target.**
   - For a composite request (today: concatenate every target pass's pings
     into one `drape_pass()` call, `sidescan_viewer_window.cpp:1533-1553`),
     instead: build each pass's own `SidescanTexture` + mesh + UVs (A.2-A.3,
     independently per pass — untouched by other passes), then render all
     of them with a **top-down orthographic** camera into one shared
     offscreen framebuffer sized to a chosen ground resolution (a
     `composite_texel_m` parameter — the operator-facing knob equivalent to
     today's `max_nodes`/CUBE-cell tuning), using a `QOpenGLFramebufferObject`
     with:
     - Colour attachment 0: RGB amplitude-colour (LUT-mapped, same shader
       logic as A.4's fragment shader).
     - Colour attachment 1 (MRT): a small state code (0 = unseen, 1 =
       shadow, 2 = painted) — needed because the tri-state today's CPU loop
       computes per node (`sidescan_viewer_window.cpp:1178-1195`) must
       survive being decided per-fragment by a depth test now, and GL
       guarantees all colour attachments for a draw are written or
       discarded together under the same depth test, so this rides for
       free alongside colour attachment 0.
     - Depth attachment: **not real depth — the per-fragment `score`**
       (see B.2), with `glDepthFunc(GL_GREATER)` so the higher-scoring
       pass's fragment wins the ground pixel. Cleared to `0.0` (below any
       real score, so first-touch always wins over "nothing painted").
   - This produces one ground-aligned composite raster (RGB colour + state
     code), which is then applied to the 3D mesh via a **third**, simpler
     UV scheme: planar `(x - origin_x) / extent_x, (y - origin_y) /
     extent_y` — axis-aligned world coordinates, no per-ping projection
     needed at this stage, since the hard part (which pass's imagery wins
     each ground pixel) is already resolved in the offscreen pass.

**B.2. Score, reproducing today's semantics exactly.**
   - `score = ping_score(row) * range_score(u, mode)` where `u =
     slant/max_slant` computed from the UV's column against that ping's
     `row_max_slant`/`row_sample0_slant` (A.2) — **this is the identical
     formula** `drapePing()` computes today (`sidescan_drape.cpp:190-193`),
     including both `RangeScoreMode::Nearest` and `::MidRange` variants
     (`sidescan_drape.hpp:82` enum, `76-81` doc comment).
   - Extract the formula into a pure, header-only function — `inline float
     texel_score(float ping_straightness_score, double u, RangeScoreMode
     mode)` in `sidescan_geometry.hpp` — shared by: (a) CPU unit tests
     (fast, no GL — see C), (b) whatever per-ping/per-row uniform data
     feeds the shader. The **shader-side** GLSL reimplementation cannot
     call this C++ function directly, so it must be transcribed by hand
     into GLSL and is documented in the shader source as "mirrors
     `texel_score()` in sidescan_geometry.hpp — keep in sync" — a
     transcription bug is still caught because C's integration tests (C.2)
     exercise the real GPU path, not just the CPU formula.
   - Shadow fragments write `state=1`, a fixed near-black colour, and a
     **very low but nonzero** score (e.g. `1e-6`, safely below the real
     score floor of `ping_score_min * 0.05` from the existing `range_score`
     clamp, `sidescan_drape.cpp:191-193`) — reproduces today's "amplitude
     always beats shadow" rule (`if (!isfinite(amplitude)) shadow = 1`,
     conceptually) while still letting a *later* real pass's shadow
     mark register over an *earlier* pass's total silence (score `0.0`,
     the clear value) at the same ground pixel.

### C. Testability — risk and mitigation

**Risk** (operator-flagged, and confirmed true by B.1/B.2 above): moving
conflict resolution onto the GPU (a depth test across draw calls) is
materially harder to unit-test than a CPU function call, and the existing
CPU composite tests exercise exactly the mechanism this plan removes
(`drape_pass()`'s per-cell `score` compare over concatenated pings).

**Mitigation**:

- **C.1. Split what can stay CPU-testable from what cannot.** `texel_score()`
  (B.2) and the box-average column builder (A.2) are pure functions with no
  GL dependency — both get direct CPU unit tests, fast and exact, covering
  the actual arithmetic the GPU path is supposed to reproduce.
- **C.2. Port, don't drop, the two existing composite tests** — as GPU
  readback tests, following the precedent in
  `test/test_point_cloud_view.cpp` (`gl_available()` self-skip,
  `QOffscreenSurface`, `grabFramebuffer()`/pixel readback, the
  `MultiPassColour` test's "which colour family dominates which region"
  assertion shape). New file `test/test_sidescan_composite.cpp` (or folded
  into `test_point_cloud_view.cpp` — decide at implementation based on
  fixture reuse), with `CMakeLists.txt`'s `ENVIRONMENT
  "QT_QPA_PLATFORM=offscreen;LIBGL_ALWAYS_SOFTWARE=1"` property added to
  its `ament_add_gtest` target (currently only `test_point_cloud_view` has
  it):
  - `StraightPassBeatsTurningPassInComposite` (ported): same two synthetic
    pass fixtures as today's CPU test (straight-running amplitudes = sample
    index; mid-turn constant 999), driven through the full B.1/B.2 pipeline
    into an offscreen composite, then `grabFramebuffer()`/read back the
    conflict cell's colour and assert it matches the straight pass's LUT
    colour, not the turning pass's. **Fails today** because the new
    entrypoints (composite FBO builder) don't exist yet — this is a
    from-scratch GPU test for new code, not a regression guard against an
    existing bug, and that framing is stated explicitly in the test's
    comment so a future reader doesn't mistake it for one.
  - `RangeScoreModeFlipsConflicts` (ported): same near/far synthetic pings,
    both `RangeScoreMode` variants, asserting the composite's winning
    colour flips between modes exactly as today's CPU test does. Same
    "fails today because it's new" framing.
  - **New**: `CrossPingBilinearBlendIsNonDegenerate` — the direct answer to
    Plan Review Finding 1. Two adjacent same-channel pings with markedly
    different constant amplitudes (e.g. 0.2 and 0.8, normalized); assert
    the rendered colour at a ground point *exactly halfway between them*
    is **strictly between** the two pings' LUT colours (not equal to
    either). This fails against any per-cell winner-take-all mechanism
    (today's `drape_pass()`, or the prior plan's inert step 3) by
    construction, since winner-take-all always returns one ping's exact
    value.
  - **New**: `ShadowBoundaryStaysColumnSharp` — a synthetic terrain with an
    occluding rise placed so the true shadow boundary falls strictly
    between two adjacent texture columns (not aligned to any CUBE-cell
    boundary); assert the rendered shadow/lit transition in the composite
    lands at the column-resolution position, not snapped to the (coarser)
    CUBE-cell grid. This is the direct regression guard for "shadows must
    stay sharp at sample resolution," and fails against a hypothetical
    per-vertex-visibility implementation (the alternative the operator
    explicitly rejected) by construction.
  - **New**: `TilingSplitsOversizedPass` — a synthetic pass with more rows
    than a (test-injected, small) max-texture-size stand-in; assert the
    builder produces multiple tiles rather than one oversized texture
    request, and that the composite result is unaffected by the tile
    boundary (a target placed exactly on a tile seam still resolves
    correctly). Fails today trivially (no tiling code exists).
- **C.3. Gate-sync regression (carried over from the prior plan, retargeted).**
  A ping that `ping_is_drapable()` (A.5) rejects must be excluded from the
  texture builder's row list **and** from any coverage/UV-assignment
  bounds computation, exactly as `extend_surface_for_drape()` already
  excludes it from grid growth today. Since A.5 unifies the gate into one
  shared helper, this becomes one test against `ping_is_drapable()` plus
  one integration check that each of the three call sites (`drapePing()`,
  `extend_surface_for_drape()`, the new texture builder) actually calls it
  rather than reimplementing the condition — a regression guard against
  reintroducing the `fe8b2d7` bug class in the new code path specifically.

## Files to Change

| File | Change |
|------|--------|
| `src/sidescan_geometry.hpp` | Add `tx_beamwidth_rad`; add `resolve_reported_beamwidth()` (inline, header-only — no `.cpp` exists); add `ping_is_drapable()`; add `texel_score()`. |
| `src/sidescan_texture.hpp`/`.cpp` (**new**) | `SidescanTexture` struct + per-pass, per-channel texture builder (rows/cols, box-averaged columns, per-row `ping_score`/range inputs, shadow march at column resolution). |
| `src/sidescan_drape.hpp`/`.cpp` | `drapePing()`/`drape_pass()` and `extend_surface_for_drape()` call the shared `ping_is_drapable()` instead of their own inline copies of the gate; header-comment updated (drops "nearest sample, no averaging," describes the texture-mapped kernel and its cross-ping blend). Terrain-growing role of `extend_surface_for_drape()` is otherwise unchanged. |
| `src/cube_lab.hpp`/`.cpp` | New `build_cube_mesh_uv()` sibling of `build_cube_mesh_colored()` — same triangulation, UV output instead of RGB. |
| `src/point_cloud_view.hpp`/`.cpp` | New `setSurfaceTexture()` entrypoint, second shader program (or `u_use_texture` switch), amplitude/shadow/LUT textures, `GL_LINEAR`/`GL_NEAREST` filtering split, tiling for oversized passes, offscreen `QOpenGLFramebufferObject` support for the composite path (B.1). |
| `src/sidescan_viewer_window.cpp` | Sidescan-shade path (~1126-1206) rewired to build UV mesh + upload textures instead of the CPU `node_rgb` loop; composite path (`requestDrape()`, ~1487-1560) rewired to per-pass FBO rendering instead of concatenated-pings `drape_pass()`. |
| `test/test_sidescan_texture.cpp` (**new**) | Pure-CPU tests: box-average dilution (ported, dilution-aware per Plan Review Finding 2), `texel_score()` (both `RangeScoreMode` variants), `ping_is_drapable()` gate-sync (C.3), beamwidth convention/fallback (ported from prior plan, A.1), **new**: `resolve_reported_beamwidth()` rejects a degrees-magnitude value (e.g. `25.0f`, ~0.44 deg-in-along-track-terms mistakenly left as a bare degrees number — well above the `1.2` rad bound) as not-reported, alongside the existing empty/all-zero/non-finite cases. |
| `test/test_sidescan_composite.cpp` (**new**, or folded into `test_point_cloud_view.cpp`) | GPU readback tests (C.2): ported `StraightPassBeatsTurningPassInComposite`, ported `RangeScoreModeFlipsConflicts`, new `CrossPingBilinearBlendIsNonDegenerate`, new `ShadowBoundaryStaysColumnSharp`, new `TilingSplitsOversizedPass`. |
| `test/test_sidescan_drape.cpp` | Existing `StraightPassBeatsTurningPassInComposite`/`RangeScoreModeFlipsConflicts` **removed** here once ported (they test a mechanism this plan replaces) — a comment left in their place pointing at the new location, per the workspace's "explain removals" norm rather than a silent deletion. |
| `.agents/README.md` | **New** subsection under `marine_perception_tools`'s inventory row describing the texture-mapped drape (not a correction — Plan Review Finding 3 confirmed no existing text describes the drape kernel there today). |

## Test plan summary

See Approach C for the full list and per-test "what makes it fail today"
justification. Every new/ported test is either (a) a pure-CPU test of an
extracted formula (`texel_score`, box-average, `ping_is_drapable`) with no
GL dependency, or (b) a GPU readback test following the `grabFramebuffer()`
precedent in `test_point_cloud_view.cpp`, gated the same way (self-skip
without a usable GL context, `LIBGL_ALWAYS_SOFTWARE` in CI).

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Documentation accuracy / verify against source | All file/line references above were re-verified against current source on `feature/issue-39` (not assumed from the prior plan) — including correcting the prior plan's wrong `sidescan_geometry.cpp` reference. |
| Fix completely, not "good enough" | The redesign addresses the review's must-fix (Finding 1) at the architecture level rather than patching the inert weighted-average; it also folds in the issue's deferred step 3 rather than leaving a known-inert half-measure in place. |
| Test-driven | Every new numeric/GPU behaviour gets a dedicated test, split CPU/GPU per C.1 specifically so the parts that *can* be fast and exact are, and the GPU-only parts are still covered end-to-end. |
| A change includes its consequences | LUT colour-mapping moving CPU->GPU, the two existing composite tests needing porting (not silent deletion), and issue #34's adjacency are all called out explicitly rather than discovered later. |

## ADR Compliance

No ADR in `docs/decisions/` in this project repo governs sonar geometry,
resampling, or GPU rendering architecture; none triggered.

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `PingGeometry` (new field) | Every construction site (`test/test_sidescan_drape.cpp`, `test/test_sidescan_geometry.cpp`, new `test_sidescan_texture.cpp`) | Yes — default-construction preserves `tx_beamwidth_rad = 0.0` (not-reported fallback), so only the new tests need to construct it explicitly. |
| `drapePing()`/`extend_surface_for_drape()`'s inline gate -> shared `ping_is_drapable()` | Both call sites, plus the new texture builder | Yes — this is the direct fix for the Issue Review's gate-sync action item, done by eliminating the duplication rather than re-syncing comments. |
| `PointCloudView::setSurface()` per-vertex-colour ceiling | `setSurfaceTexture()` added alongside, not replacing — the CUBE `Depth`/`Uncertainty`/`Intensity` shade modes keep using `setSurface()` unchanged | Yes. |
| CPU LUT colour-mapping loop (`sidescan_viewer_window.cpp:1178-1195`) moving to a GPU shader | The manual-range/auto-range UI wiring immediately after it (`cube_srange_` spinboxes) still needs `lo`/`hi` values to display and to feed the shader as uniforms — logic preserved, just relocated | Yes — covered in A.4. |
| Two existing CPU composite tests test a removed mechanism | Ported (C.2), not silently deleted; a comment left at their old location | Yes. |
| `.agents/README.md` | New drape-kernel subsection | Yes. |
| Issue #34 (`setSurface` validation gap) | New `setSurfaceTexture()` must not be a *worse* validation gap than today's `setSurface()` | Yes — explicitly scoped as "same posture, not a fix" in Context. |

## Documentation & Instruction Impact

- **Stale docs**: `sidescan_drape.hpp`'s header comment (`sidescan_drape.hpp:18-29`,
  "Nearest sample per cell, no averaging") must be rewritten to describe the
  texture-mapped kernel — this file's comment is the only place in the repo
  that currently makes this claim (per Plan Review Finding 3, `.agents/README.md`
  has none today).
- `.agents/README.md` needs a **new** subsection (not a correction — nothing
  existing describes the drape kernel there).
- **Agent-instruction candidates** (proposal only, not part of this PR): the
  GPU-depth-test-as-conflict-resolution pattern (B.1/B.2) is a reusable
  technique if any other pass-compositing need arises in this workspace;
  worth a one-line pointer in `.agent/knowledge/` if it recurs. Flagged for
  the operator, not actioned here.

## Open Questions

- [ ] `texture_col_pitch_m` default (replaces the prior plan's
  `drape_cell_m` open question) — near the measured GCV across-track
  sample spacing (~0.02 m) keeps the box-average close to lossless; a
  coarser default trades memory/perf for column resolution. Operator call.
- [ ] `composite_texel_m` default for the ground-space composite FBO (B.1)
  — analogous knob, separate from the per-pass column pitch since the
  composite raster is axis-aligned world space, not sonar-native. Operator
  call.
- [ ] Non-uniform sample counts across a pass's pings (A.2) — assumed rare/
  absent per the ground-truth measurements, but not verified against a bag
  with a mid-recording parameter change. Needs a source check before
  implementation locks in the "pad shorter rows" behaviour as sufficient.
- [ ] Second shader program vs. a `u_use_texture` uniform switch on the
  existing program (A.4) — implementation-level choice, no behavioural
  difference; left open pending whichever is cleaner once the attribute
  layout is drafted.
- [ ] Whether `test_sidescan_composite.cpp` is its own file or folds into
  `test_point_cloud_view.cpp` (C.2) — depends on how much fixture code
  (synthetic pings/terrain) it shares with `test_sidescan_texture.cpp`
  versus `test_point_cloud_view.cpp`'s GL scaffolding; decide once both
  are drafted.

**Resolved by this revision** (previously open, now moot): triangular vs.
Gaussian along-track kernel shape — there is no hand-rolled along-track
kernel anymore; cross-ping blending is GPU bilinear filtering across
texture rows, not a weighted accumulation over offsets.

## Estimated Scope

Single PR, larger than the prior plan's estimate: one new source file pair
(`sidescan_texture.{hpp,cpp}`), a new `PointCloudView` rendering path
(shader program, textures, offscreen FBO composite), `cube_lab`'s new UV
mesh builder, and both viewer-window call sites (single-pass shade +
composite). Two new test files plus edits to two existing ones. No
cross-repo changes. Given the size, consider at implementation time whether
A (single-pass texture path) and B (GPU composite) split cleanly into two
PRs after all, once A's shader/texture plumbing exists as a concrete
diff — noted as an implementation-time judgment call, not decided here,
since A.2's texture builder and A.5's shared gate are load-bearing for both.
