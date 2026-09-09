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
  middle-click seek, contact marking with Contact-store save/load + GeoJSON
  export). `--start/--end` (UNIX ns or ISO-8601 UTC) cue the scrub to that
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
  modifiers: **left-drag** draws or replaces the region, **left-click** clears
  it, **middle-click** centres the view on the point (and seeks the time cursor
  there when a bag is open), **middle-drag** pans, and the wheel zooms. While
  *Mark contact* is on, that visible mode takes left-drag for marking.

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
map — plus **track**/**grid**/**coast** declutter toggles (the coast
toggle's state persists across restarts). The cloud legend's rows
have **checkboxes** to show/hide individual passes, and **clip to
contact** (+ margin) restricts a multi-pass load to the selected contact's
neighbourhood. Contacts can be deleted from the list's context menu.

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
