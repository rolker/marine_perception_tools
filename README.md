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
costmaps, with a menu bar, frame scrubber, and live parameter dock. Editing a
knob re-simulates the loaded window and refreshes the regenerated costmap.

```bash
# Open empty, then File → Open Bag…
ros2 run marine_perception_tools sea_surface_tuner

# …or open a *_ffmpeg_seg bag on startup
ros2 run marine_perception_tools sea_surface_tuner <bag_uri> \
    [--start-s S] [--end-s S] [--window-m 120] [--res 0.25] \
    [--max-range 150] [--min-grazing-deg 0] [--probe]
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
- **Scrubber**: seek through the loaded window (forward seeks accumulate
  incrementally; rewinds re-simulate from the window start).
- **Parameter dock** — the live-tunable `sea_surface_segmentation` knobs
  ([unh_marine_perception#22](https://github.com/rolker/unh_marine_perception/issues/22)):
  - occupancy: `decay_half_life_s`, `lethal_threshold`, `obstacle_clamp`,
    `clear_floor`, `free_threshold`
  - accumulation: `max_range`, `min_grazing_angle_deg`, `obstacle_prob_min`,
    `max_evidence_step`

  Invalid values are rejected (status bar) and reverted. Window geometry
  (`--window-m`/`--res`) is CLI-only (it sizes the buffer at construction).
- **`--probe`** loads the bag, prints seg/RGB/costmap counts (and the first RGB
  frame's size + channel means), and exits — a headless check of the loader,
  incl. H.265 decode, against a real bag. Run under `QT_QPA_PLATFORM=offscreen`.

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
