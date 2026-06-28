---
issue: 13
---

# Issue #13 — Export the Qt-free contact-builder as a linkable target

## Implementation
**Status**: complete
**When**: 2026-06-28
**By**: Claude Code Agent (Claude Opus)

**Branch**: feature/issue-13 at `1c2161b`
**Scope**: CMake-only refactor — no behavior change, no source edits to
`contact_store.{hpp,cpp}` (same code, recompiled into a new target).

### What changed (`CMakeLists.txt`)
- **New exported target `contact_builder`** compiling `src/contact_store.cpp`
  (owns `MapPoint`, `make_box_contact`, `GeoJsonExportResult` /
  `export_contacts_geojson`, and class `ContactStore`). Verified Qt-free: the
  header pulls only `marine_interfaces/msg/contact.hpp`; the `.cpp` uses
  `marine_interfaces` + `rclcpp` (CDR serialization in `ContactStore::save/load`).
  Built `POSITION_INDEPENDENT_CODE ON` so the static lib can be linked into a
  SHARED rqt plugin downstream (mirrors `marine_tiled_raster_store`).
- `add_library(marine_perception_tools::contact_builder ALIAS contact_builder)`
  for in-tree namespaced use.
- **Removed** `src/contact_store.cpp` from `sidescan_core` (it now compiles only
  `sidescan_bag_session.cpp` + `coverage_raster.cpp`).
- `sidescan_target_viewer` now also links `contact_builder` (its
  `sidescan_viewer_window.cpp` uses `ContactStore`).
- `test_contact_store` links `contact_builder` (the target that now owns
  `make_box_contact`) instead of `sidescan_core`.
- **Export idiom** (mirrors `core_ws` `marine_tiled_raster_store`):
  - `install(FILES src/contact_store.hpp DESTINATION include/marine_perception_tools)`
  - `target_include_directories(contact_builder PUBLIC $<BUILD_INTERFACE:.../src> $<INSTALL_INTERFACE:include>)`
    — in-tree consumers keep the flat `#include "contact_store.hpp"`; downstream
    uses the namespaced `#include "marine_perception_tools/contact_store.hpp"`.
  - `install(TARGETS contact_builder EXPORT export_marine_perception_tools ...)`
  - `ament_export_targets(export_marine_perception_tools HAS_LIBRARY_TARGET)`
  - `ament_export_dependencies(marine_interfaces rclcpp)`

  Note: Jazzy's `ament_target_dependencies` does **not** accept `PUBLIC`/`PRIVATE`
  keywords, so both deps are linked via the plain form
  `ament_target_dependencies(contact_builder marine_interfaces rclcpp)`. `rclcpp`
  must be exported because the static lib propagates it to consumers as a
  `$<LINK_ONLY:...>` interface link.

`package.xml` needed no change — `marine_interfaces` and `rclcpp` are already
`<depend>`s.

### Build / test result
- Lower layers had to be built first (clean worktree): `underlay_ws` (22 pkgs),
  `core_ws` (35 pkgs), `sensors_ws` up to `sea_surface_segmentation`, then
  `ui_ws` `marine_colormap` + `marine_sonar_widgets`.
- `./ui_ws/build.sh marine_perception_tools` — **OK**.
- `./ui_ws/test.sh marine_perception_tools` — **231 tests, 0 errors, 0 failures,
  37 skipped** (the skipped are the offscreen-GL `test_point_cloud_view` set on a
  headless box). `test_contact_store`: 7/7 pass.
- Lint on the touched file: `lint_cmake`, `cpplint`, `uncrustify`, `copyright`
  all 0 failures.
- Verified the install tree: `marine_perception_tools::contact_builder` is a
  `STATIC IMPORTED` target with `IMPORTED_LOCATION lib/libcontact_builder.a`,
  `INTERFACE_INCLUDE_DIRECTORIES include`, and `INTERFACE_LINK_LIBRARIES`
  resolving `marine_interfaces` + `rclcpp::rclcpp`; header present at
  `include/marine_perception_tools/contact_store.hpp`.

### For the downstream #86 link
A consumer (`rqt_operator_tools#86`) links the contact-builder with:

```cmake
find_package(marine_perception_tools REQUIRED)
# ...
target_link_libraries(<your_target> marine_perception_tools::contact_builder)
# or, in an ament target:
ament_target_dependencies(<your_target> marine_perception_tools)
```

- **find_package name**: `marine_perception_tools`
- **target name**: `marine_perception_tools::contact_builder`
- **public include**: `#include "marine_perception_tools/contact_store.hpp"`
  (namespace `marine_perception_tools` — `MapPoint`, `make_box_contact`,
  `export_contacts_geojson`, `GeoJsonExportResult`, `ContactStore`)
- Add `<depend>marine_perception_tools</depend>` to the consumer's `package.xml`.
- Transitive deps `marine_interfaces` and `rclcpp` are exported, so the consumer
  does not need to `find_package` them just to link `contact_builder`.

No scope creep into #86's marking feature; this change is purely the export.
