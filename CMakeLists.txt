cmake_minimum_required(VERSION 3.8)
project(marine_perception_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(ffmpeg_encoder_decoder REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(marine_autonomy REQUIRED)
find_package(marine_colormap REQUIRED)
find_package(marine_contacts REQUIRED)
find_package(marine_interfaces REQUIRED)
find_package(marine_sonar_widgets REQUIRED)
find_package(marine_survey_index REQUIRED)
find_package(marine_tiled_raster_store REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sea_surface_segmentation REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Qt is sourced version-agnostically so a future Jazzy -> L-turtle (Qt6) bump is
# mechanical: Jazzy ships Qt5, the next ROS LTS moves rqt/rviz to Qt6. We build
# against Qt5 today (the only Qt on the Jazzy operator station) but reference the
# toolkit through Qt${QT_VERSION_MAJOR} targets so the version number is not
# hard-coded anywhere below the find_package line.
set(QT_VERSION_MAJOR 5)
# Concurrent: background window loads (QtConcurrent::run + QFutureWatcher) keep
# the GUI responsive while a multi-second window reload/warm-up runs.
# Gui + OpenGL: the shared marine_sonar_widgets panes (WaterfallWidget,
# EchogramWidget) and the new 3D point-cloud viewport are QOpenGLWidgets.
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Concurrent Gui OpenGL)
set(CMAKE_AUTOMOC ON)

# Headless core (no Qt): bag loading + the re-sim engine. Kept Qt-free so the
# engine is unit-testable without a display, and so the breakable logic
# (path-dependent re-sim) is exercised in CI.
set(CORE_DEPS
  builtin_interfaces
  cv_bridge
  ffmpeg_encoder_decoder
  ffmpeg_image_transport_msgs
  grid_map_core
  image_geometry
  nav_msgs
  rclcpp
  rosbag2_cpp
  sea_surface_segmentation
  sensor_msgs
  tf2
  tf2_msgs
)
add_library(tuner_core
  src/bag_loader.cpp
  src/resim_engine.cpp
)
target_include_directories(tuner_core PUBLIC src)
target_link_libraries(tuner_core ${OpenCV_LIBRARIES})
ament_target_dependencies(tuner_core ${CORE_DEPS})

add_executable(sea_surface_tuner
  src/main.cpp
  src/main_window.cpp
)
target_link_libraries(sea_surface_tuner
  tuner_core
  Qt${QT_VERSION_MAJOR}::Widgets
  Qt${QT_VERSION_MAJOR}::Concurrent)

# The Qt-free Contact builder (make_box_contact / ContactStore /
# export_contacts_geojson) used to live here as an exported `contact_builder` target
# (issue #13). It now has a lightweight home in the marine_contacts package
# (unh_marine_autonomy#243) so downstream consumers (rqt_operator_tools#86) link it
# without pulling this package's offline-tuner dependency chain. The offline sidescan
# viewer links marine_contacts::marine_contacts below.

# Sidescan viewer core (no Qt): bag ingest + pure projection geometry for the
# offline georeferenced sidescan viewer / target tracker (issue #7). Kept Qt-free
# and separate from tuner_core so the projection math is unit-tested without a
# display, mirroring the tuner_core split.
set(SIDESCAN_CORE_DEPS
  builtin_interfaces
  geographic_msgs
  marine_acoustic_msgs
  marine_interfaces
  rclcpp
  rosbag2_cpp
  sensor_msgs
  tf2
  tf2_msgs
)
add_library(sidescan_core
  src/sidescan_bag_session.cpp
  src/coverage_raster.cpp
  src/mbes_window_reader.cpp
)
target_include_directories(sidescan_core PUBLIC src)
ament_target_dependencies(sidescan_core ${SIDESCAN_CORE_DEPS})

# Headless ingest verification (the sidescan analogue of --probe).
add_executable(sidescan_probe src/sidescan_probe.cpp)
target_link_libraries(sidescan_probe sidescan_core)

# Headless survey-index bridge (#19): map click -> pass list, reusing the
# marine_survey_index query library. Qt-free so it unit-tests without a display.
add_library(survey_index_bridge src/survey_index_bridge.cpp)
target_include_directories(survey_index_bridge PUBLIC src)
target_link_libraries(survey_index_bridge SQLite::SQLite3)
# marine_autonomy is used directly (marine_autonomy/gggs.h) as well as
# transitively via marine_survey_index; depend on it explicitly.
ament_target_dependencies(survey_index_bridge marine_autonomy marine_survey_index)

# Qt app: the survey explorer (#24; grew out of the offline sidescan viewer /
# target tracker, issues #7/#8) — bag scrub viewer + survey-index explorer in
# one window.
add_executable(survey_explorer
  src/survey_explorer_main.cpp
  src/sidescan_viewer_window.cpp
  src/sidescan_canvas.cpp
  src/point_cloud_view.cpp
  src/mbes_pass_loader.cpp
  src/pass_timeline_widget.cpp
)
target_link_libraries(survey_explorer
  sidescan_core
  survey_index_bridge
  marine_contacts::marine_contacts
  Qt${QT_VERSION_MAJOR}::Widgets
  Qt${QT_VERSION_MAJOR}::Concurrent
  Qt${QT_VERSION_MAJOR}::Gui
  Qt${QT_VERSION_MAJOR}::OpenGL)
ament_target_dependencies(survey_explorer
  marine_autonomy marine_colormap marine_sonar_widgets marine_tiled_raster_store)

# Back-compat shim (#22): the old executable name execs the explorer.
add_executable(sidescan_target_viewer src/sidescan_target_viewer_shim.cpp)

install(TARGETS sea_surface_tuner sidescan_probe survey_explorer sidescan_target_viewer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_resim_engine test/test_resim_engine.cpp)
  target_link_libraries(test_resim_engine tuner_core)
  ament_target_dependencies(test_resim_engine
    image_geometry sea_surface_segmentation sensor_msgs)

  # Pure span-policy logic (buffer_policy.hpp) — header-only, no bag/Qt/ROS, so
  # the test needs only the source include dir.
  ament_add_gtest(test_buffer_policy test/test_buffer_policy.cpp)
  target_include_directories(test_buffer_policy PRIVATE src)

  # Pure sidescan projection geometry (sidescan_geometry.hpp) — header-only, no
  # bag/Qt/ROS, so the test needs only the source include dir.
  ament_add_gtest(test_sidescan_geometry test/test_sidescan_geometry.cpp)
  target_include_directories(test_sidescan_geometry PRIVATE src)

  # Pure MBES (M3) beam geometry (mbes_geometry.hpp) — header-only; needs the
  # source include dir + marine_acoustic_msgs for the SonarDetections message.
  ament_add_gtest(test_mbes_geometry test/test_mbes_geometry.cpp)
  target_include_directories(test_mbes_geometry PRIVATE src)
  ament_target_dependencies(test_mbes_geometry marine_acoustic_msgs)

  # Pure rolling-distance scrub policy (distance_buffer_policy.hpp) — header-only.
  ament_add_gtest(test_distance_buffer_policy test/test_distance_buffer_policy.cpp)
  target_include_directories(test_distance_buffer_policy PRIVATE src)

  # Coverage raster (quality-wins paint + coverage skip) — links sidescan_core.
  ament_add_gtest(test_coverage_raster test/test_coverage_raster.cpp)
  target_link_libraries(test_coverage_raster sidescan_core)

  # Jump-to-pass time→distance mapping (distance_interval) — synthetic index,
  # no bag I/O; links sidescan_core (#17).
  ament_add_gtest(test_sidescan_interval test/test_sidescan_interval.cpp)
  target_link_libraries(test_sidescan_interval sidescan_core)

  # Survey-overview geo<->pixel projection (pure header) — correctness-critical
  # cos(lat) mapping (#19).
  ament_add_gtest(test_survey_projection test/test_survey_projection.cpp)
  target_include_directories(test_survey_projection PRIVATE src)

  # Survey-index bridge: click -> pass rows against a file-backed fixture DB
  # written through the marine_survey_index schema (#19); tile-selection and
  # nav-track accessors (#24).
  ament_add_gtest(test_survey_index_bridge test/test_survey_index_bridge.cpp)
  target_link_libraries(test_survey_index_bridge survey_index_bridge SQLite::SQLite3)

  # Pure tile-selection math (tile_selection.hpp, #24) — header-only.
  ament_add_gtest(test_tile_selection test/test_tile_selection.cpp)
  target_include_directories(test_tile_selection PRIVATE src)

  # Pure gap-compressed timeline axis (pass_timeline_model.hpp, #24) —
  # header-only.
  ament_add_gtest(test_pass_timeline_model test/test_pass_timeline_model.cpp)
  target_include_directories(test_pass_timeline_model PRIVATE src)

  # Multi-pass windowed-read pure parts (#21): cross-bag frame reprojection +
  # the shared tf_lift helpers. No bag I/O; links sidescan_core.
  ament_add_gtest(test_mbes_window_reader test/test_mbes_window_reader.cpp)
  target_link_libraries(test_mbes_window_reader sidescan_core)

  # The contact-store test moved to the marine_contacts package (unh_marine_autonomy#243)
  # along with the builder it exercises.

  # MBES 3D point-cloud viewport smoke test (offscreen GL; self-skips headless).
  # Compiles the unit-under-test source directly + Qt OpenGL.
  ament_add_gtest(test_point_cloud_view
    test/test_point_cloud_view.cpp
    src/point_cloud_view.cpp)
  target_include_directories(test_point_cloud_view PRIVATE src)
  ament_target_dependencies(test_point_cloud_view marine_acoustic_msgs marine_colormap)
  target_link_libraries(test_point_cloud_view
    Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Gui Qt${QT_VERSION_MAJOR}::OpenGL)
  set_tests_properties(test_point_cloud_view PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;LIBGL_ALWAYS_SOFTWARE=1")

  # Pure map-ENU -> geo anchor probe (map_geo_anchor.hpp, #24) — header-only.
  ament_add_gtest(test_map_geo_anchor test/test_map_geo_anchor.cpp)
  target_include_directories(test_map_geo_anchor PRIVATE src)

  # Robust basemap contrast (basemap_contrast.hpp, #24) — header-only. Pins
  # the desk-verify bug: store outlier cells must not own the colour scale.
  ament_add_gtest(test_basemap_contrast test/test_basemap_contrast.cpp)
  target_include_directories(test_basemap_contrast PRIVATE src)

  # Multi-pass cloud loader (#24): error paths (missing bag, empty input) —
  # the happy path needs a real bag and is covered by test_mbes_window_reader's
  # units plus the window smoke below. QtCore only (QStringList notes).
  ament_add_gtest(test_mbes_pass_loader
    test/test_mbes_pass_loader.cpp
    src/mbes_pass_loader.cpp)
  target_include_directories(test_mbes_pass_loader PRIVATE src)
  target_link_libraries(test_mbes_pass_loader sidescan_core Qt${QT_VERSION_MAJOR}::Core)

  # Survey-explorer window smoke (#24; offscreen GL, self-skips headless):
  # --index path constructs against a fixture DB, loads tiles + track, and
  # survives a programmatic tile selection driving the cloud loader.
  ament_add_gtest(test_survey_explorer_window
    test/test_survey_explorer_window.cpp
    src/sidescan_viewer_window.cpp
    src/sidescan_canvas.cpp
    src/point_cloud_view.cpp
    src/mbes_pass_loader.cpp
    src/pass_timeline_widget.cpp)
  target_include_directories(test_survey_explorer_window PRIVATE src)
  ament_target_dependencies(test_survey_explorer_window
    marine_autonomy marine_colormap marine_sonar_widgets marine_tiled_raster_store)
  target_link_libraries(test_survey_explorer_window
    sidescan_core
    survey_index_bridge
    marine_contacts::marine_contacts
    SQLite::SQLite3
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Concurrent
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::OpenGL)
  set_tests_properties(test_survey_explorer_window PROPERTIES
    ENVIRONMENT "QT_QPA_PLATFORM=offscreen;LIBGL_ALWAYS_SOFTWARE=1")

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
