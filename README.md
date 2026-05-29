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

### Status — Milestone A (forward-camera viewer + live re-tuning)

Implemented: the forward camera (`oak_forward`) segmentation is replayed beside
the re-simulated costmap, with a frame scrubber and a parameter dock. Editing a
knob re-simulates the loaded window and refreshes the costmap.

```bash
ros2 run marine_perception_tools sea_surface_tuner <bag_uri> \
    [--start-s S] [--end-s S] [--window-m 120] [--res 0.25] \
    [--max-range 150] [--min-grazing-deg 0]
```

- **Panes**: `oak_forward` segmentation (`rgb8`) | re-simulated lethal grid
  (boat-centred, N-up, log-odds palette).
- **Scrubber**: seek through the loaded frame window (forward seeks accumulate
  incrementally; rewinds re-simulate from the window start).
- **Parameter dock** (Milestone-A, #22-stable knobs): `decay_half_life_s`,
  `lethal_threshold`, `max_range`, `min_grazing_angle_deg`. Invalid values are
  rejected (status bar) and reverted.

The segmentation that drives the costmap is the raw `rgb8` `Image` on
`/bizzy/sensors/cameras/oak_forward/segmentation` (no ffmpeg decode); camera pose
and boat pose are resolved from TF (`bizzy/map_tide` → optical / `base_link`).

**Milestone B** (after
[rolker/unh_marine_perception#22](https://github.com/rolker/unh_marine_perception/issues/22)
Phase 1 lands its new parameter model): the dock's knob table extends to #22's
per-pixel-log-odds / asymmetric-clamp / reflex-gate parameters, making this a
#22 *tuning* harness. The 4-camera mosaic and recorded-costmap overlay remain
future work. See
[#1](https://github.com/rolker/marine_perception_tools/issues/1).

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
