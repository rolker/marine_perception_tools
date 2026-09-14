# marine_perception_tools

Operator-station tools for marine perception. Lives in the `ui_ws` layer
(operator station only) so its Qt dependency stays off the boat-side host.

## Packages

| Package | Description |
|---------|-------------|
| `marine_perception_tools` | Build target for `sea_surface_tuner` and `survey_explorer` (below). |

## sea_surface_tuner

An offline C++/Qt application to **replay recorded bags and tune the sea-surface
costmap marking** against them. It reuses the *real* algorithm via the exported
`sea_surface_segmentation` core headers + shared bag→costmap driver
([rolker/unh_marine_perception#23](https://github.com/rolker/unh_marine_perception/issues/23)),
so what you tune offline matches what runs on the boat.

Motivated by deployment
[rolker/unh_echoboats_project11#186](https://github.com/rolker/unh_echoboats_project11/issues/186)
(small-buoy marking under dark skies) and doubles as the validation harness for
[rolker/unh_marine_perception#22](https://github.com/rolker/unh_marine_perception/issues/22).

### Status — interactive 4-camera tuner

All four OAK cameras are fused and replayed beside the recorded and re-simulated
costmaps, with a menu bar, whole-bag time scrubber, and a parameter dock with an
Apply/Reset workflow.

```bash
# Open empty, then File → Open Bag…
ros2 run marine_perception_tools sea_surface_tuner

# …or open a *_ffmpeg_seg bag on startup
ros2 run marine_perception_tools sea_surface_tuner <bag_uri> \
    [--window-m 120] [--res 0.25] [--max-range 150] [--min-grazing-deg 0] \
    [--integration-halflives 1] [--margin-s 10] [--retention-s 120] \
    [--start-s S] [--end-s S] [--probe]
```

- **Menu bar** — *File → Open Bag…* (open a rosbag2 directory at runtime;
  re-tunable without relaunching), *File → Quit*. The window can open empty.
- **Layout** — three rows in a vertical splitter that fills the window; drag the
  handles to give each row more or less height. Costmap panes carry a boat marker
  (a heading arrow at panel centre, since the panes are boat-centred / N-up).
  - Row 1 — camera RGB (H.265 `image_raw/ffmpeg`, decoded): port | fwd | stbd | aft
  - Row 2 — segmentation masks: port | fwd | stbd | aft
  - Row 3 — recorded costmap (`/bizzy/local_costmap/costmap`) | regenerated
    (tuned) costmap, both boat-centred / N-up / 1:1 for direct comparison
- **Cameras fused into one costmap.** All four cameras' segmentation frames feed
  a single `OccupancyBuffer`, exactly as the deployed `SeaSurfaceLayer` fuses its
  observation sources — so the tuned result matches the boat.
- **Segmentation source** is the raw `rgb8` `Image` when present, else the
  `.../segmentation/compressed` `CompressedImage` (size-trimmed bags); the UI
  notes the fallback (a JPEG-compressed mask corrupts the obstacle-probability
  channel). Camera/boat pose resolved from TF (`bizzy/map_tide` → optical /
  `base_link`).
- **Windowed File → Open.** Opening a bag scans it once (TF cache, camera
  models, time bounds) but buffers only a *window* of frames around the scrub
  point — never the whole recording (a long 4-camera bag would exhaust memory).
  The window is `integration-halflives × decay_half_life_s` of accumulation
  warm-up behind the playhead, plus `margin-s` of reload-free scrub slack on each
  side; up to `retention-s` of already-read frames beyond it are kept so nearby
  re-visits avoid a re-read.
- **Scrubber** spans the whole bag in time. Seeks *inside* the loaded window are
  instant — backward seeks restore a cached buffer checkpoint and replay only the
  short tail (forward seeks accumulate incrementally). A seek *outside* the
  window loads a fresh window around the target on a **background thread** so the
  GUI never freezes (deferred to slider release — a drag doesn't trigger a reload
  mid-motion). The load runs in **two stages**: the camera RGB, segmentation, and
  recorded costmap appear as soon as the window is read (panes grey out while
  loading so a stale view isn't mistaken for the target), then the slow
  regenerated costmap fills in once its warm-up replay finishes (it shows
  "computing costmap…" until then). A newer scrub supersedes an in-flight load.
- **Fidelity caveat (windowed ≠ full history).** Because the regenerated costmap
  is warmed only over the window (not the whole bag), cells the boat observed
  *before* the window started read as unobserved — so near the window edge / for
  revisited areas the regenerated costmap can differ from the recorded one for
  reasons that aren't a parameter effect. The status bar shows the warm-up
  actually behind the current frame (`windowed: warm-up N s …`); near the bag
  start it is shorter than the full integration.
- **Parameter dock** — the tunable `sea_surface_segmentation` knobs
  ([unh_marine_perception#22](https://github.com/rolker/unh_marine_perception/issues/22)):
  - occupancy: `decay_half_life_s`, `lethal_threshold`, `obstacle_clamp`,
    `clear_floor`, `free_threshold`
  - accumulation: `max_range`, `min_grazing_angle_deg`, `obstacle_prob_min`,
    `max_evidence_step`

  Hover any knob (label or value box) for a tooltip explaining what it does.
  Editing a knob does **not** re-simulate immediately — a full warm-up replay is
  too expensive per keystroke. An edited-but-unapplied knob is highlighted;
  **Apply** validates the whole batch (an invalid value is rejected instantly to
  the status bar and the batch is left unapplied) then re-simulates once **on a
  background thread** so the GUI stays responsive ("applying…" while it runs), and
  **Reset** reverts edits to the applied values. Applied tuning is preserved
  across window reloads. Window geometry (`--window-m`/`--res`) is CLI-only (it
  sizes the buffer at construction).
- **`--probe`** loads the `[--start-s, --end-s]` range, prints seg/RGB/costmap
  counts (and the first RGB frame's size + channel means), and exits — a headless
  check of the loader, incl. H.265 decode, against a real bag. Run under
  `QT_QPA_PLATFORM=offscreen`, and **always with a `--start-s/--end-s` clamp**
  (`--probe` does a one-shot load, not the windowed read, so an unclamped probe
  buffers the whole bag).

The `*_ffmpeg_seg` bags under `~/data/logs/bizzy_images/` are self-contained
(all four cameras' RGB + segmentation + camera_info, `/tf`, and the recorded
costmap), so File → Open targets one such bag.

**Future work**: `nav2_overlay.yaml` parameter export (save tuned values back
out). See [#1](https://github.com/rolker/marine_perception_tools/issues/1).

## survey_explorer

An offline C++/Qt **survey-data explorer** for georeferenced sidescan and MBES
recordings (grew out of the sidescan target viewer,
[#7](https://github.com/rolker/marine_perception_tools/issues/7)/[#8](https://github.com/rolker/marine_perception_tools/issues/8)/[#24](https://github.com/rolker/marine_perception_tools/issues/24)).
One window, two modes that compose:

> **What it is and what constrains it:** [`docs/survey_explorer.md`](docs/survey_explorer.md).
> **Where it is going:** [#36](https://github.com/rolker/marine_perception_tools/issues/36).
> (The original umbrella, [unh_marine_autonomy#258](https://github.com/rolker/unh_marine_autonomy/issues/258),
> is closed — its five stages are all merged.)

- **Bag mode** — `survey_explorer <bag> [--start T --end T]`: scrub a
  recording along distance travelled; the rolling window paints georeferenced
  sidescan coverage on the map beside the slant-range waterfall, MBES
  backscatter, water-column echogram, and 3D point cloud (linked cursor,
  middle-click to recentre and seek, contact marking with Contact-store
  save/load + GeoJSON export). `--start/--end` (UNIX ns or ISO-8601 UTC) cue the scrub to that
  time window once indexing completes.
- **Explorer mode** — `survey_explorer --index survey_index.db [--stores DIR]`:
  the map becomes the survey index — store-tile basemap (GGGS GeoTIFFs;
  `--stores` defaults to `<index dir>/depths/processed`), per-bag nav track
  with direction arrows, and the selectable index-tile grid. Left-drag on the
  map draws the **region** (#42), the map's one geographic selection: its exact
  bounds are the processing extent for CUBE, and every `mbes-bathy` pass of the
  index tiles it covers loads into
  the 3D cloud (one golden-angle colour per pass, legend beside the pane,
  cross-bag reprojection through the `earth` anchor), and the **time bar**
  under the scrub controls fills with the selection's passes.
  Map gestures (#42) are one region and one navigation button, with no
  modifiers: **left-drag** draws or replaces the region, **middle-click**
  centres the view on the point (and seeks the time cursor there when a bag is
  open), **middle-drag** pans, and the wheel zooms. A **left-click** with no
  drag never touches the region — a click that silently threw the region away
  was the one destructive thing it could do, so clearing is asked for by name:
  **right-click** the map for its context menu and choose **Clear Selection**,
  which is greyed out when there is nothing selected. The same menu also
  offers **Copy Position**, which puts the latitude and longitude of the point
  you right-clicked on the clipboard — the point the menu was opened at, not
  wherever the pointer ended up on its way down the menu — in the status row's
  own format, so what you paste is what you read. It is greyed out wherever
  the map cannot place that point at all (no survey index and a bag with no
  earth reference), rather than copying a zero. That menu is where further map
  actions will appear. While *Mark contact* is on, that visible mode takes
  left-drag for marking.

  What a bare left-click *does* do is **cue the time bar from the map** (#46).
  **Hover** within a few pixels of a nav track and the nearest fix on it is
  marked with a yellow disc, with the time the boat was there in the status
  row beside the lat/lon (in whichever zone the **UTC** toggle is showing).
  Click it and the scrub and the trackline views cue to that instant, by the
  same path a committed time on the time bar takes. This is the map answering
  the one question the time bar cannot — *when did **that** pass happen* —
  which is what you need in an area worked over many times, where the passes
  span several recordings and you should not have to know which. The hit
  radius is in **screen pixels**, not ground metres, so "close enough" means
  the same thing at every zoom. Beyond it nothing highlights, and the
  highlight stays out of the way of every other gesture: no marker during a
  region drag, a pan, a recentre glide, a zoom, or contact marking. With no
  highlighted fix the click is still the complete no-op #42 made it.

  The **geographic readout** in the status row follows the cursor over *every*
  spatial pane (#47), not just the map: the 3D cloud, both waterfalls and the
  echogram each convert the position in their own frame — the waterfalls and
  the echogram through the open bag's earth anchor, the 3D pane through the
  frame its points were loaded in, which for a selection or CUBE run may be
  another recording's. Each position is shown with the name of the pane that
  produced it (`MBES 3D  43.020305, -71.360000`), because with four panes
  feeding one label a number nobody can attribute is a number you cannot act
  on. A pane that cannot place the cursor — a recording with no earth
  reference, a waterfall pixel with no pose behind it — shows *nothing* rather
  than a zero that reads like a position, and the readout clears when the
  cursor leaves a pane, so a position on screen is always a live one.

  Cueing to an instant in a recording that is not open reopens and re-indexes
  that whole bag, and the status line says so while it happens. In a revisited
  area most clicks land in another recording and pay it: underneath, the
  cursor the trackline views follow is a distance along the open bag's track,
  not an absolute time, and only the time bar spans every recording. Promoting
  that cursor to absolute time is the larger piece of work, tracked against
  #36. What the reopen does **not** do is move the map: a recording opened as
  a side effect of cueing a time leaves the zoom and centre you were working
  at exactly where they were, in both the 3D pane and the map. The view is
  fitted to a new recording only when you asked for that recording by name —
  *File → Open Bag…* or a bag on the command line.
  The recentre **glides** to the point over about three quarters of a second,
  easing in and out, so the eye can follow the map across instead of having to
  re-find the survey after a jump. The seek is not delayed by it — the time
  cursor moves at the click. Any gesture that touches the view takes over at
  once: a pan or a wheel zoom leaves the glide where it stands, and a second
  middle-click retargets it from there rather than cancelling.

  Underneath all of it is a built-in **world coastline** so a collection-wide
  view is navigable at all: Natural Earth 1:50m, public domain, vendored into
  the package (`data/coastline/`) and never fetched — the build and the
  application touch the network nowhere. It is **orientation, not
  navigation**: generalised to the kilometre, drawn under every real layer,
  and faded out entirely before survey zoom, where the store imagery and the
  index carry the answer. Never navigate by it.
  Both modes together (`--index` + a bag argument) place the open bag's
  coverage on the survey map through its earth anchor.

The **time bar** (ported from GeoZui4D's TimeControl; navigation-only for
now) is how you go to a specific time: a zoomable tape with a
multi-resolution tick ladder (ms → years) and the current time at a centre
cursor, over a full-extent scrollbar whose translucent thumb shows the
visible window. Drag the tape to pan time (pull vertically to stretch the
scale), wheel to zoom (ctrl = coarse, shift = fine), double- or middle-click
to jump, drag the thumb or click the trough to page through the extent.
Committing a time cues the scrub there — switching bags through the index
selection when the time falls in another recording. Clicking a pass bar
cues that pass directly.

More explorer controls: the Map pane header carries a **basemap layer
picker** (bathymetry / backscatter / sidescan store layers discovered next
to the index) and a **basemap colormap** combo — contrast is
percentile-scaled per layer so residual store outliers cannot blank the
map. The map overlays are switched from the **View menu**, which holds
**Nav Track** (`Ctrl+1`), **Survey Index Tile Grid** (`Ctrl+2`), **World
Coastline** (`Ctrl+3`) and **Measuring Grid (metres)** (`Ctrl+4`); all four
are greyed out until an index is open. Two of those are grids and they are
different layers: **Survey Index Tile Grid** switches the cyan outlines of
the ~54 m survey **index tiles** (the things you select to load passes),
while **Measuring Grid (metres)** switches the slate Cartesian ruler —
lines at the spacing set by the Grid box in the bottom row, labelled in
metres from the map origin. The measuring grid is a ruler for sizing a
target on one bag, so it is **on for a bag and off when an index is
open**, where the origin is arbitrary and the lines are just noise; the
Grid spacing box greys out while it is hidden. The **World Coastline** and
**Measuring Grid** toggles remember their state across restarts (**Nav
Track** and **Survey Index Tile Grid** do not). The cloud legend's rows
have **checkboxes** to show/hide individual passes, and **clip to
contact** (+ margin) restricts a multi-pass load to the selected contact's
neighbourhood. Contacts can be deleted from the list's context menu.

The **3D pane's colouring** is one vocabulary shared by the point cloud and the
CUBE surface: **Depth**, **Uncertainty**, **Backscatter**, **Pass** and
**Sidescan**, in that order in both selectors. **Pass** is an ordinary entry —
loading a multi-pass region defaults to it, but you can leave it for a scalar
ramp and come back; it is not a mode that takes the selector away. An entry a
layer cannot carry stays in the list, greyed, with the reason on the entry
rather than silently missing: the points offer no **Uncertainty** (not every
cloud carries one yet — soundings loaded for the CUBE lab now carry real
error-model variances, the scrub window's own cloud does not, and a channel
that measured different things on different clouds would be worse than none;
it opens when mpt#56 swaps the second path over) and no
**Sidescan** (that is a drape painted onto CUBE nodes by marching the terrain),
and the surface offers no **Pass** (a node merges every pass that touched it).

### Per-sounding uncertainty in the lab

CUBE weights every sounding by its own uncertainty, and the lab supplies a real
one (#55): soundings loaded for the CUBE lab are projected through
`cube::DetectionsProjector` and `cube::ErrorModel` — the same code the boat
runs live and the same code `import_bag`, `batch_regen_bag` and
`bag_to_geotiff` run offline — so each sounding arrives with the vertical and
horizontal variances the error model computed from the ping, the platform
attitude and the heave. The angle-aware placeholder that used to stand in for
them is gone. This path is the CUBE lab's multi-pass selection load only; the
scrub window's own cloud still uses the simpler one-sound-speed projection and
leaves both variances unset, which is why the **Uncertainty** point channel
stays greyed until mpt#56.

The error model needs a survey configuration the bag does not carry, so the
lab runs the **library defaults** — and says so, in one caveat shown both in
*params…* and in the load note. In order of consequence:

1. A generic **2 m GPS drms** is assumed for every sounding, so no sounding's
   horizontal variance falls below 4 m². This is not a lab shortcut: the live
   projector and all three offline `cube_bathymetry` tools run the same
   default, so the lab agrees with the store instead of inventing a tighter
   number. It does **not** smear the surface — `Parameters::influenceRadius`
   subtracts the 99% horizontal term from the depth-budget term and floors
   what is left at the cell size, so at the cell sizes the lab runs each
   sounding still influences one cell; the ~5.15 m radius that 4 m² allows is
   a ceiling, reached only at coarse cells under a loose vertical budget.
2. Lever arms are zero and the device is generic, so the M3's across-track
   beamwidth falls back to the device default — and since
   `kongsberg_em_bridge` currently reports no per-beam beamwidths
   (marine_tools#85), that fallback applies to every beam. The load note
   carries the library's own warning line, with the count.
3. Speed over ground is unavailable offline and passed as NaN, which the error
   model floors to zero. The speed-dependent latency terms drop out, so the
   horizontal error is **optimistic** for a moving vessel.

The general fix — a survey-configuration record the live node publishes and
every offline tool reads — is unh_marine_autonomy#385.

None of this corrects the refraction smile: that is a *systematic* error,
which no uncertainty model removes — see #28. A sounding the error model
cannot give a usable uncertainty (a missing attitude or heave transform will
do it) is skipped rather than given an invented one, and the run's note says
how many were, pointing at the load note where the causes are counted.

A **Run CUBE** *adds* its surface over the soundings already loaded instead of
replacing them: after a run you are still looking at your selection, now with a
surface over it. The one exception is a run whose reference world frame differs
from the loaded cloud's — the surface is a grid in its own frame and would be
placed by luck over soundings in another — where the run falls back to showing
its own soundings and says so in the status line.

The **cell size** runs from 0.001 m to 50 m at three decimals, stepping by a
centimetre. That floor is just the finest spacing the box can display — it is
not a judgement about what is worth gridding, and it is not there to bound
memory: how large a grid a run may allocate is the separate **max grid nodes**
limit in *params…*, which asks (with the real node count and a one-shot
override) rather than refusing. Cells far below the beam footprint — a
2-degree beam in 5 m of water footprints about 0.17 m at nadir — resolve the
sounding pattern rather than the seafloor, which is worth knowing and is the
operator's call to make.

A cell size below the floor is corrected to the nearest value the box accepts
**and reported in the status line** (#42). Qt's own behaviour is to restore
the value that was in the box *before* the edit, on focus-out — which is the
moment you click **Run CUBE** — so a cell size typed below the old 0.02 m
floor ran at whatever had been there before, with nothing on screen saying the
entry had been dropped. Only the floors do this: text above a maximum cannot
grow into anything valid, so Qt refuses those keystrokes and the digit
visibly never appears.

Bag opens are accelerated by a **bag-index cache** (`--cache-dir`, default
`$XDG_CACHE_HOME/survey_explorer`): the whole-bag metadata scan runs once per
bag and is then a file read, keyed by the bag's size+mtime (bags are never
written to). Pre-build the whole campaign with
`survey_explorer --index survey_index.db --warm-cache` (headless: set
`QT_QPA_PLATFORM=offscreen`), after which any time-bar or timeline cue opens
its bag in about a second.

The index is produced by `marine_survey_index` (unh_marine_autonomy); see its
`docs/survey_index_schema.md` for the schema contract.

`sidescan_target_viewer` remains as a **compatibility shim** that execs
`survey_explorer` with the same arguments
([#22](https://github.com/rolker/marine_perception_tools/issues/22)).

## Qt version

Built against **Qt5** — the only Qt on the ROS 2 Jazzy operator station. The
CMake references the toolkit through `Qt${QT_VERSION_MAJOR}` targets so a future
move to Qt6 (the next ROS LTS, on Ubuntu 26.04) is a one-line bump rather than a
rewrite.

## Build

This repo is checked out into `ui_ws` via the workspace layer manifest. From the
workspace:

```bash
make build
```

Standalone (from a sourced Jazzy environment with the package in a colcon
workspace):

```bash
colcon build --packages-select marine_perception_tools --symlink-install
```

## License

Apache-2.0. See [`LICENSE`](LICENSE).
