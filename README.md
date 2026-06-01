# marine_perception_tools

Operator-station tools for marine perception. Lives in the `ui_ws` layer
(operator station only) so its Qt dependency stays off the boat-side host.

## Packages

| Package | Description |
|---------|-------------|
| `marine_perception_tools` | Build target for `sea_surface_tuner` (below). |

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
- **Layout**:
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
  window loads a fresh window around the target; this is deferred to slider
  release (a drag doesn't trigger a multi-second reload mid-motion) and shows a
  "Warming up…" status while it loads.
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

  Editing a knob does **not** re-simulate immediately — a full warm-up replay is
  too expensive per keystroke. An edited-but-unapplied knob is highlighted;
  **Apply** validates the whole batch and re-simulates once (an invalid value is
  rejected to the status bar and the batch is left unapplied), and **Reset**
  reverts edits to the applied values. Window geometry (`--window-m`/`--res`) is
  CLI-only (it sizes the buffer at construction).
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
