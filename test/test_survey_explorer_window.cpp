// Copyright 2026 Roland Arsenault
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Survey-explorer window smoke (#24): the --index path constructs against a
// fixture DB written through the marine_survey_index schema, places the index
// layers, and survives a programmatic tile selection driving the multi-pass
// cloud loader (whose bag is deliberately missing — the error path must
// surface in the legend/status, never crash). Offscreen GL; self-skips when
// even software GL is unavailable. (The executable rename to survey_explorer
// lands in phase e; this test already carries the explorer name.)

#include <gtest/gtest.h>

#include <QApplication>
#include <QLabel>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QThread>
#include <QTreeWidget>

#include <sqlite3.h>

#include <cstdio>
#include <set>
#include <stdexcept>
#include <string>

#include "marine_autonomy/gggs.h"
#include "marine_survey_index/schema.hpp"
#include "sidescan_canvas.hpp"
#include "sidescan_viewer_window.hpp"
#include "time_bar_widget.hpp"

namespace
{

using marine_perception_tools::SidescanCanvas;
using marine_perception_tools::SidescanViewerWindow;
using marine_perception_tools::TimeBarWidget;

constexpr double kLat = 43.02;
constexpr double kLon = -71.36;

bool gl_available()
{
  QSurfaceFormat fmt;
  fmt.setRenderableType(QSurfaceFormat::OpenGL);
  fmt.setVersion(3, 3);
  QOffscreenSurface surface;
  surface.setFormat(fmt);
  surface.create();
  if (!surface.isValid()) {
    return false;
  }
  QOpenGLContext ctx;
  ctx.setFormat(fmt);
  if (!ctx.create() || !ctx.makeCurrent(&surface)) {
    return false;
  }
  ctx.doneCurrent();
  return true;
}

QApplication & app()
{
  static int argc = 1;
  static char arg0[] = "test_survey_explorer_window";
  static char * argv[] = {arg0, nullptr};
  static QApplication a(argc, argv);
  return a;
}

// Spin the event loop until `done` holds or ~5 s pass (worker completions are
// delivered as queued signals, so plain waiting would never dispatch them).
template<typename Pred>
bool process_until(Pred && done)
{
  for (int i = 0; i < 250; ++i) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    if (done()) {
      return true;
    }
    QThread::msleep(20);
  }
  return done();
}

// A file-backed fixture index: one bag (whose path does NOT exist on disk)
// with an mbes-bathy pass and, one tile north, a sidescan pass — plus a short
// nav track. Same schema the indexer writes.
class ExplorerWindowFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    db_path_ = std::string(::testing::TempDir()) + "/explorer_fixture.db";
    std::remove(db_path_.c_str());
    sqlite3 * db = marine_survey_index::openIndexDb(db_path_);
    exec(db, "INSERT INTO bags (id, path, size_bytes, mtime_ns, indexed_at_ns)"
      " VALUES (1, '/nonexistent/bag_a', 100, 200, 300);");
    const auto tile = gggs::Level(14).gridIndex(kLat, kLon);
    exec(db, ("INSERT INTO passes (bag_id, level, tile_row, tile_col, sensor_type,"
      " topic, t_start_ns, t_end_ns, ping_count) VALUES (1, 14, " +
      std::to_string(tile.row()) + ", " + std::to_string(tile.column()) +
      ", 'mbes-bathy', '/detections', 1000, 2000, 42);").c_str());
    const auto north_tile = gggs::Level(14).gridIndex(kLat + 0.01, kLon);
    exec(db, ("INSERT INTO passes (bag_id, level, tile_row, tile_col, sensor_type,"
      " topic, t_start_ns, t_end_ns, ping_count) VALUES (1, 14, " +
      std::to_string(north_tile.row()) + ", " + std::to_string(north_tile.column()) +
      ", 'sidescan-port', '/ss_port', 5000, 6000, 7);").c_str());
    exec(db, ("INSERT INTO nav_track (bag_id, t_ns, latitude, longitude) VALUES"
      " (1, 1000, " + std::to_string(kLat) + ", " + std::to_string(kLon) + "),"
      " (1, 2000, " + std::to_string(kLat + 1e-4) + ", " + std::to_string(kLon) + ");")
      .c_str());
    sqlite3_close(db);
  }

  void TearDown() override
  {
    std::remove(db_path_.c_str());
  }

  static void exec(sqlite3 * db, const char * sql)
  {
    char * err = nullptr;
    const int rc = sqlite3_exec(db, sql, nullptr, nullptr, &err);
    const std::string msg = err ? err : "unknown sqlite error";
    sqlite3_free(err);
    ASSERT_EQ(rc, SQLITE_OK) << msg;
  }

  std::string db_path_;
};

TEST_F(ExplorerWindowFixture, MissingIndexThrows)
{
  app();   // the QApplication must exist before any surface/widget, including the GL probe
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  EXPECT_THROW(
    window.openSurveyIndex("/nonexistent/survey_index.db", "/nonexistent/stores"),
    std::runtime_error);
}

TEST_F(ExplorerWindowFixture, IndexModeSurvivesTileSelectionAndClear)
{
  app();   // before the GL probe (same reason as above)
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  // Empty stores dir: the explorer must work from the index alone (stage-2
  // lesson); the fit falls back to the index extent.
  window.openSurveyIndex(db_path_, std::string(::testing::TempDir()));
  window.show();
  QCoreApplication::processEvents();

  auto * canvas = window.findChild<SidescanCanvas *>();
  ASSERT_NE(canvas, nullptr);
  ASSERT_TRUE(canvas->hasGeoOrigin());

  auto * legend = window.findChild<QTreeWidget *>("cloud_legend");
  ASSERT_NE(legend, nullptr);
  EXPECT_FALSE(legend->isVisible());
  auto * timeline = window.findChild<TimeBarWidget *>("time_bar");
  ASSERT_NE(timeline, nullptr);
  // Always-on in index mode: the bar spans the campaign (the fixture's nav
  // track) from startup, before any selection.
  EXPECT_TRUE(timeline->isVisible());
  EXPECT_TRUE(timeline->hasExtent());
  EXPECT_EQ(timeline->passCount(), 0);

  // Select both indexed tiles: the mbes-bathy pass heads for the cloud; its
  // bag is missing, so the loader's error path must land in the legend (one
  // zero-count row) and the window must stay alive. The time bar shows BOTH
  // passes (mbes + sidescan).
  canvas->selectTiles({0, 1});
  ASSERT_TRUE(process_until([legend]() {return legend->topLevelItemCount() > 0;}))
    << "cloud load never completed";
  EXPECT_TRUE(legend->isVisible());
  ASSERT_EQ(legend->topLevelItemCount(), 1);
  EXPECT_EQ(legend->topLevelItem(0)->text(1), "0");
  EXPECT_TRUE(timeline->isVisible());
  EXPECT_EQ(timeline->passCount(), 2);

  // Clearing the selection hands the pane back to scrub mode; the campaign
  // time bar stays (only its pass bars clear).
  canvas->clearTileSelection();
  QCoreApplication::processEvents();
  EXPECT_FALSE(legend->isVisible());
  EXPECT_EQ(legend->topLevelItemCount(), 0);
  EXPECT_TRUE(timeline->isVisible());
  EXPECT_EQ(timeline->passCount(), 0);
}

}  // namespace
