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
#include <QImage>
#include <QLabel>
#include <QElapsedTimer>
#include <QMouseEvent>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QThread>
#include <QTreeWidget>
#include <QWheelEvent>

#include <sqlite3.h>

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <set>
#include <utility>
#include <stdexcept>
#include <string>

#include "coastline_data.hpp"
#include "marine_autonomy/gggs.h"
#include "marine_survey_index/schema.hpp"
#include "sidescan_canvas.hpp"
#include "sidescan_viewer_window.hpp"
#include "time_bar_widget.hpp"

namespace
{

using marine_perception_tools::Coastline;
using marine_perception_tools::GeoRect;
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

// Render the canvas over a geographic box with the coastline layer on or off.
// The visibility toggle is applied LAST because it invalidates the static
// layer cache: a plain zoom change would otherwise be served by the canvas's
// scaled-blit fast path instead of a real repaint.
QImage renderOverBox(
  SidescanCanvas & canvas, double half_span_deg, bool coastline_on)
{
  canvas.fitGeo(kLat - half_span_deg, kLon - half_span_deg,
    kLat + half_span_deg, kLon + half_span_deg);
  canvas.setCoastlineVisible(coastline_on);
  return canvas.grab().toImage();
}

// The scale rule of #41, at the pixel level: the coastline is orientation, so
// it must be visible when the operator is looking at a region and must
// contribute NOTHING once they are at survey scale — where it would be wrong
// by hundreds of metres and would read as chart detail. Toggling it must then
// change nothing at all on screen.
TEST_F(ExplorerWindowFixture, CoastlineDrawsAtRegionZoomAndVanishesAtSurveyZoom)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanCanvas canvas;
  canvas.resize(400, 300);
  canvas.setGeoOrigin(kLat, kLon);

  // A synthetic coastline through the fixture's position, so it is in view at
  // both zooms and only the scale rule can remove it.
  Coastline coastline;
  marine_perception_tools::CoastlinePolyline line;
  for (int i = -20; i <= 20; ++i) {
    line.points.emplace_back(kLat + 0.002 * i, kLon + 0.001 * i * i);
  }
  line.south = line.points.front().first;
  line.north = line.points.back().first;
  line.west = kLon;
  line.east = kLon + 0.4;
  coastline.lines.push_back(line);
  canvas.setCoastline(coastline);

  // ~1 degree across the 400 px canvas: hundreds of metres per pixel, the
  // zoom at which nothing else tells the operator where they are.
  const QImage region_on = renderOverBox(canvas, 0.5, true);
  const QImage region_off = renderOverBox(canvas, 0.5, false);
  EXPECT_NE(region_on, region_off) << "the coastline never drew at region zoom";

  // ~200 m across the same canvas: survey scale.
  const QImage survey_on = renderOverBox(canvas, 0.001, true);
  const QImage survey_off = renderOverBox(canvas, 0.001, false);
  EXPECT_EQ(survey_on, survey_off) << "the coastline still draws at survey zoom";
}

// --- middle-click recentre (#42) --------------------------------------------

// The canvas has no view-centre accessor, so read the settled view the way
// the LOD basemap does and take the middle of it.
std::pair<double, double> viewCentreGeo(const SidescanCanvas & canvas)
{
  const auto region = canvas.visibleGeoRegion();
  EXPECT_TRUE(region.has_value());
  if (!region) {
    return {0.0, 0.0};
  }
  return {(region->south + region->north) * 0.5, (region->west + region->east) * 0.5};
}

// Where a screen pixel sits geographically, from the visible region alone:
// the canvas plane is equirectangular, so the mapping is linear in both axes.
std::pair<double, double> pixelGeo(const SidescanCanvas & canvas, const QPoint & pos)
{
  const GeoRect r = canvas.visibleGeoRegion().value();
  return {
    r.north - (r.north - r.south) * pos.y() / canvas.height(),
    r.west + (r.east - r.west) * pos.x() / canvas.width()};
}

void middleClick(SidescanCanvas & canvas, const QPoint & pos)
{
  const QPointF local(pos);
  const QPointF global(canvas.mapToGlobal(pos));
  QMouseEvent press(
    QEvent::MouseButtonPress, local, global, Qt::MiddleButton, Qt::MiddleButton,
    Qt::NoModifier);
  QApplication::sendEvent(&canvas, &press);
  QMouseEvent release(
    QEvent::MouseButtonRelease, local, global, Qt::MiddleButton, Qt::NoButton,
    Qt::NoModifier);
  QApplication::sendEvent(&canvas, &release);
}

// A canvas with a geographic frame, laid out and fitted over a box around the
// fixture position. grab() forces the pending fit to apply (it only lands on a
// laid-out paint), so the view is settled before the gesture under test.
void prepareFittedCanvas(SidescanCanvas & canvas)
{
  canvas.resize(400, 300);
  canvas.setGeoOrigin(kLat, kLon);
  canvas.fitGeo(kLat - 0.01, kLon - 0.01, kLat + 0.01, kLon + 0.01);
  canvas.grab();
}

// Zero duration is the headless/test contract: the recentre is instant, so no
// snapshot or pixel test can land on an intermediate frame.
TEST_F(ExplorerWindowFixture, MiddleClickRecentresInstantlyAtZeroDuration)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanCanvas canvas;
  prepareFittedCanvas(canvas);
  canvas.setRecenterDurationMs(0);

  const QPoint target_px(100, 60);
  const auto target = pixelGeo(canvas, target_px);
  int view_changed = 0;
  QObject::connect(
    &canvas, &SidescanCanvas::viewChanged, &canvas, [&view_changed]() {++view_changed;});

  middleClick(canvas, target_px);

  EXPECT_FALSE(canvas.recenterAnimating());
  const auto centre = viewCentreGeo(canvas);
  EXPECT_NEAR(centre.first, target.first, 1e-9);
  EXPECT_NEAR(centre.second, target.second, 1e-9);
  EXPECT_EQ(view_changed, 1) << "the settled view must announce itself exactly once";
}

// With a duration the view GLIDES: it has not arrived when the click returns,
// and viewChanged is held until it settles (the LOD basemap must re-evaluate
// once, for the view the operator ends up looking at).
TEST_F(ExplorerWindowFixture, MiddleClickGlidesAndSettlesOnTheClickedPoint)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanCanvas canvas;
  prepareFittedCanvas(canvas);
  EXPECT_EQ(canvas.recenterDurationMs(), marine_perception_tools::kRecenterDurationMs);

  const QPoint target_px(320, 240);
  const auto start = viewCentreGeo(canvas);
  const auto target = pixelGeo(canvas, target_px);
  int view_changed = 0;
  QObject::connect(
    &canvas, &SidescanCanvas::viewChanged, &canvas, [&view_changed]() {++view_changed;});

  middleClick(canvas, target_px);

  ASSERT_TRUE(canvas.recenterAnimating());
  EXPECT_EQ(view_changed, 0) << "viewChanged fired before the view settled";
  // Barely any of the run has elapsed, and the curve starts flat, so the view
  // is still essentially where it was: it did not jump.
  const auto in_flight = viewCentreGeo(canvas);
  EXPECT_NEAR(in_flight.first, start.first, 1e-6);
  EXPECT_NEAR(in_flight.second, start.second, 1e-6);

  // Land it deterministically instead of racing the timer with a sleep.
  canvas.finishRecenterNow();
  EXPECT_FALSE(canvas.recenterAnimating());
  EXPECT_EQ(view_changed, 1);
  const auto settled = viewCentreGeo(canvas);
  EXPECT_NEAR(settled.first, target.first, 1e-9);
  EXPECT_NEAR(settled.second, target.second, 1e-9);
}

// The animation must never fight the operator: a wheel zoom mid-glide takes
// the view over from where the glide stands, and the abandoned target must
// not land afterwards.
TEST_F(ExplorerWindowFixture, AWheelZoomMidGlideTakesOverTheView)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanCanvas canvas;
  prepareFittedCanvas(canvas);

  const QPoint target_px(380, 20);
  const auto start = viewCentreGeo(canvas);
  const auto target = pixelGeo(canvas, target_px);
  middleClick(canvas, target_px);
  ASSERT_TRUE(canvas.recenterAnimating());

  const QPointF centre_px(canvas.width() / 2.0, canvas.height() / 2.0);
  QWheelEvent wheel(
    centre_px, QPointF(canvas.mapToGlobal(centre_px.toPoint())), QPoint(0, 0),
    QPoint(0, 120), Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
  QApplication::sendEvent(&canvas, &wheel);

  EXPECT_FALSE(canvas.recenterAnimating()) << "the glide outlived the gesture that took over";
  // Zooming about the widget centre holds the centre point, so the view is
  // still where the glide was abandoned — near the start, not at the target.
  const auto after = viewCentreGeo(canvas);
  EXPECT_NEAR(after.first, start.first, 1e-6);
  EXPECT_NEAR(after.second, start.second, 1e-6);
  EXPECT_GT(std::abs(after.second - target.second), 1e-5);
}

// A second middle click RETARGETS: the gesture means "go to here", so a fresh
// one means "actually, here". It restarts from wherever the map has reached,
// and the first target is forgotten.
TEST_F(ExplorerWindowFixture, ASecondMiddleClickRetargetsTheGlide)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanCanvas canvas;
  prepareFittedCanvas(canvas);

  const QPoint first_px(360, 40);
  const auto first_target = pixelGeo(canvas, first_px);
  middleClick(canvas, first_px);
  ASSERT_TRUE(canvas.recenterAnimating());

  const QPoint second_px(40, 260);
  const auto second_target = pixelGeo(canvas, second_px);
  middleClick(canvas, second_px);
  ASSERT_TRUE(canvas.recenterAnimating()) << "the second click cancelled instead of retargeting";

  canvas.finishRecenterNow();
  const auto settled = viewCentreGeo(canvas);
  EXPECT_NEAR(settled.first, second_target.first, 1e-9);
  EXPECT_NEAR(settled.second, second_target.second, 1e-9);
  EXPECT_GT(std::abs(settled.second - first_target.second), 1e-5);
}

// The reason the glide is built the way it is (#42): the static layer cache
// (coastline, store basemap, measuring grid, tile grid) is keyed on the view
// centre, so animating the centre naively would re-rasterize every layer on
// every frame — a stutter on any collection-wide view. The glide rides the
// same translated blit a pan does, and rebuilds ONCE when it settles.
TEST_F(ExplorerWindowFixture, TheGlideRebuildsTheLayerCacheOnceNotPerFrame)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanCanvas canvas;
  prepareFittedCanvas(canvas);
  // A short run so the test does not spend three quarters of a second; the
  // frame path under test is the same one the full-length glide uses.
  canvas.setRecenterDurationMs(200);
  canvas.grab();

  const std::size_t rebuilds_before = canvas.layerCacheRebuildCount();
  middleClick(canvas, QPoint(340, 250));
  ASSERT_TRUE(canvas.recenterAnimating());

  // Drive the animation: processEvents runs the frame timer, grab() forces
  // each repaint (the canvas is never shown, so paints do not arrive on
  // their own).
  int frames = 0;
  QElapsedTimer bound;
  bound.start();
  while (canvas.recenterAnimating() && bound.elapsed() < 5000) {
    QCoreApplication::processEvents();
    canvas.grab();
    ++frames;
  }
  ASSERT_FALSE(canvas.recenterAnimating()) << "the glide never settled";
  EXPECT_GT(frames, 5) << "too few frames to say anything about per-frame cost";
  canvas.grab();   // the settled repaint, where the one rebuild belongs

  EXPECT_EQ(canvas.layerCacheRebuildCount() - rebuilds_before, 1u)
    << "the layer cache was rebuilt " << canvas.layerCacheRebuildCount() - rebuilds_before
    << " times across " << frames << " animation frames";
}

}  // namespace
