# Agent Guide: marine_perception_tools

> Operator-station tools for marine perception; hosts the `sea_surface_tuner`
> offline bag-replay + segmentation→costmap tuning app.

## Workflow

**When this repo is checked out as part of a
[ROS 2 Agent Workspace](https://github.com/rolker/ros2_agent_workspace)**,
workflow rules (worktree vs. field mode, branch naming, etc.) are defined in the
workspace `AGENTS.md`. Determine the active mode before editing, from the
workspace root:

```bash
.agent/scripts/field_mode.sh --describe layers/main/ui_ws/src/marine_perception_tools
```

**Standalone use** (this repo cloned alone): only this repo's own conventions
apply.

## Package Inventory

| Package | Language | Description |
|---------|----------|-------------|
| `marine_perception_tools` | C++ (Qt5) | Builds `sea_surface_tuner`. Currently a placeholder Qt window; the tuner UI is built out per [#1](https://github.com/rolker/marine_perception_tools/issues/1). |

## Repository Layout

```
marine_perception_tools/
├── src/
│   ├── main.cpp          # QApplication entry point
│   ├── main_window.hpp   # MainWindow (QMainWindow subclass)
│   └── main_window.cpp
├── CMakeLists.txt        # ament_cmake; Qt sourced via Qt${QT_VERSION_MAJOR}
├── package.xml
├── .github/workflows/ci.yml
└── .pre-commit-config.yaml
```

## Build & Test

```bash
# From the ui_ws layer workspace
colcon build --packages-select marine_perception_tools --symlink-install
# Testing requires setup.bash in the same shell
source ../../../.agent/scripts/setup.bash && colcon test --packages-select marine_perception_tools && colcon test-result --verbose
```

Tests are currently lint-only (`ament_lint_common`); functional tests arrive
with the tuner MVP.

## Common Pitfalls

- **Qt5, not Qt6.** Jazzy's operator station is Qt5-only. The CMake is written
  version-agnostically (`Qt${QT_VERSION_MAJOR}`) so the eventual Qt6 bump (next
  ROS LTS) is mechanical — but do not assume Qt6 headers/APIs are available here.
- **ui_ws is operator-station-only** — this package and its Qt dependency must
  never become a boat-side build dependency.
- The tuner is a **standalone Qt app**, not an rqt plugin; it is not bound to
  rqt's Qt version, only to the Jazzy system Qt.
- It reuses the real algorithm from `sea_surface_segmentation`'s exported
  headers ([unh_marine_perception#23](https://github.com/rolker/unh_marine_perception/issues/23));
  that dependency is added when the tuner logic lands, not in the skeleton.
