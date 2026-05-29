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

# Qt is sourced version-agnostically so a future Jazzy -> L-turtle (Qt6) bump is
# mechanical: Jazzy ships Qt5, the next ROS LTS moves rqt/rviz to Qt6. We build
# against Qt5 today (the only Qt on the Jazzy operator station) but reference the
# toolkit through Qt${QT_VERSION_MAJOR} targets so the version number is not
# hard-coded anywhere below the find_package line.
set(QT_VERSION_MAJOR 5)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)

add_executable(sea_surface_tuner
  src/main.cpp
  src/main_window.cpp
)
target_include_directories(sea_surface_tuner PRIVATE src)
target_link_libraries(sea_surface_tuner Qt${QT_VERSION_MAJOR}::Widgets)

install(TARGETS sea_surface_tuner
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
