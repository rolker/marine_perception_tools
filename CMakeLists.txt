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
find_package(image_geometry REQUIRED)
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
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
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
target_link_libraries(sea_surface_tuner tuner_core Qt${QT_VERSION_MAJOR}::Widgets)

install(TARGETS sea_surface_tuner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_resim_engine test/test_resim_engine.cpp)
  target_link_libraries(test_resim_engine tuner_core)
  ament_target_dependencies(test_resim_engine
    image_geometry sea_surface_segmentation sensor_msgs)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
