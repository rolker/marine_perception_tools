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
find_package(marine_colormap REQUIRED)
find_package(marine_interfaces REQUIRED)
find_package(marine_sonar_widgets REQUIRED)
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
  src/contact_store.cpp
)
target_include_directories(sidescan_core PUBLIC src)
ament_target_dependencies(sidescan_core ${SIDESCAN_CORE_DEPS})

# Headless ingest verification (the sidescan analogue of --probe).
add_executable(sidescan_probe src/sidescan_probe.cpp)
target_link_libraries(sidescan_probe sidescan_core)

# Qt app: offline georeferenced sidescan viewer + target tracker (issue #7).
add_executable(sidescan_target_viewer
  src/sidescan_viewer_main.cpp
  src/sidescan_viewer_window.cpp
  src/sidescan_canvas.cpp
  src/sidescan_waterfall.cpp
)
target_link_libraries(sidescan_target_viewer
  sidescan_core
  Qt${QT_VERSION_MAJOR}::Widgets
  Qt${QT_VERSION_MAJOR}::Concurrent
  Qt${QT_VERSION_MAJOR}::Gui
  Qt${QT_VERSION_MAJOR}::OpenGL)
ament_target_dependencies(sidescan_target_viewer marine_colormap marine_sonar_widgets)

install(TARGETS sea_surface_tuner sidescan_probe sidescan_target_viewer
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

  # Contact store (box->Contact, in-box query, CDR round-trip) — links sidescan_core.
  ament_add_gtest(test_contact_store test/test_contact_store.cpp)
  target_link_libraries(test_contact_store sidescan_core)
  ament_target_dependencies(test_contact_store marine_interfaces)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
