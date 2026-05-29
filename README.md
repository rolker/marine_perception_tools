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

**Status: skeleton.** The current executable is a placeholder Qt window. The
bag-replay timeline, camera/segmentation/costmap panes, parameter widgets, and
path-dependent re-simulation described in
[#1](https://github.com/rolker/marine_perception_tools/issues/1) are layered in
on top of this foundation.

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
