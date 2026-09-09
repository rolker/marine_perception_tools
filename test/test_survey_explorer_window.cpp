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

#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QImage>
#include <QLabel>
#include <QElapsedTimer>
#include <QMenu>
#include <QMouseEvent>
#include <QPushButton>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QSettings>
#include <QSlider>
#include <QSurfaceFormat>
#include <QThread>
#include <QTreeWidget>
#include <QWheelEvent>

#include <sqlite3.h>

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <set>
#include <utility>
#include <stdexcept>
#include <string>

#include "marine_acoustic_msgs/msg/raw_sonar_image.hpp"
#include "marine_acoustic_msgs/msg/sonar_detections.hpp"
#include "rclcpp/time.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "coastline_data.hpp"
#include "cube_lab.hpp"
#include "marine_autonomy/gggs.h"
#include "marine_sonar_widgets/echogram_widget.hpp"
#include "marine_sonar_widgets/waterfall_widget.hpp"
#include "marine_survey_index/schema.hpp"
#include "color_vocabulary.hpp"
#include "point_cloud_view.hpp"
#include "sidescan_geometry.hpp"
#include "sidescan_canvas.hpp"
#include "sidescan_viewer_window.hpp"
#include "time_bar_widget.hpp"

namespace
{

using marine_perception_tools::Coastline;
using marine_perception_tools::GeoRect;
using marine_perception_tools::PointCloudView;
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
  // The window remembers operator preferences (last index, layer visibility)
  // in QSettings("UNH-CCOM", "survey_explorer"). Redirect that store into the
  // temp dir before any window exists: the suite must neither read the
  // developer's own preferences (a remembered toggle would decide a default
  // test) nor write to them.
  static const bool redirected = []() {
      QSettings::setPath(
        QSettings::NativeFormat, QSettings::UserScope,
        QString::fromStdString(std::string(::testing::TempDir()) + "/qsettings"));
      return true;
    }();
  (void)redirected;
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


// --- one colour vocabulary for the 3D pane (#36) -----------------------------

// Whether a selector row is offerable (Qt greys a disabled row but keeps it in
// the list — which is the point: a channel a layer cannot carry must still be
// visible, with its reason).
bool comboRowEnabled(const QComboBox & combo, int row)
{
  return (combo.model()->index(row, 0).flags() & Qt::ItemIsEnabled) != Qt::NoItemFlags;
}

int comboRowOf(const QComboBox & combo, const QString & text)
{
  for (int i = 0; i < combo.count(); ++i) {
    if (combo.itemText(i) == text) {
      return i;
    }
  }
  return -1;
}

// The points and the CUBE surface are two layers of one picture, so their
// selectors offer the SAME entries in the SAME order. A channel a layer cannot
// carry is present and greyed with the reason on its tooltip — never dropped,
// which is what made the two dropdowns read as unrelated controls.
TEST_F(ExplorerWindowFixture, PointAndSurfaceSelectorsShareOneColourVocabulary)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  window.show();
  QCoreApplication::processEvents();

  auto * points = window.findChild<QComboBox *>("cloud_color_combo");
  auto * surface = window.findChild<QComboBox *>("cube_shade_combo");
  ASSERT_NE(points, nullptr);
  ASSERT_NE(surface, nullptr);

  const QStringList expected{"Depth", "Uncertainty", "Backscatter", "Pass", "Sidescan"};
  ASSERT_EQ(points->count(), expected.size());
  ASSERT_EQ(surface->count(), expected.size());
  for (int i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(points->itemText(i), expected[i]);
    EXPECT_EQ(surface->itemText(i), expected[i]);
  }

  // A sounding carries no uncertainty (run_cube derives its own placeholder
  // from depth) and no sidescan amplitude (that is a drape onto CUBE nodes),
  // so both are greyed for the points — and each says why.
  for (const QString & name : {QString("Uncertainty"), QString("Sidescan")}) {
    const int row = comboRowOf(*points, name);
    ASSERT_GE(row, 0);
    EXPECT_FALSE(comboRowEnabled(*points, row)) << name.toStdString();
    EXPECT_FALSE(points->itemData(row, Qt::ToolTipRole).toString().isEmpty())
      << name.toStdString() << " is greyed without saying why";
  }
  // A CUBE node merges every pass that touched it, so Pass is the surface's
  // one greyed entry; the other three are its real shades.
  const int surface_pass_row = comboRowOf(*surface, "Pass");
  ASSERT_GE(surface_pass_row, 0);
  EXPECT_FALSE(comboRowEnabled(*surface, surface_pass_row));
  EXPECT_FALSE(
    surface->itemData(surface_pass_row, Qt::ToolTipRole).toString().isEmpty());
  for (const QString & name : {QString("Depth"), QString("Uncertainty"),
      QString("Backscatter"), QString("Sidescan")})
  {
    EXPECT_TRUE(comboRowEnabled(*surface, comboRowOf(*surface, name)))
      << name.toStdString();
  }
}

// The operator's complaint, as a test: selecting a multi-pass region must
// DEFAULT to pass colouring without taking the choice away. Pass is an
// ordinary entry — the selector stays enabled through the load, the operator
// can leave Pass for a scalar ramp and come back to it.
TEST_F(ExplorerWindowFixture, PassIsAnEntryAndTheSelectorStaysLiveThroughASelection)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  window.openSurveyIndex(db_path_, std::string(::testing::TempDir()));
  window.show();
  QCoreApplication::processEvents();

  auto * canvas = window.findChild<SidescanCanvas *>();
  auto * points = window.findChild<QComboBox *>("cloud_color_combo");
  auto * legend = window.findChild<QTreeWidget *>("cloud_legend");
  ASSERT_NE(canvas, nullptr);
  ASSERT_NE(points, nullptr);
  ASSERT_NE(legend, nullptr);

  // Before any selection the pane holds no passes, so Pass is greyed (with its
  // reason) rather than offering a colouring that would paint every point the
  // same hue.
  const int pass_row = comboRowOf(*points, "Pass");
  ASSERT_GE(pass_row, 0);
  EXPECT_FALSE(comboRowEnabled(*points, pass_row));
  EXPECT_EQ(points->currentText(), "Depth");

  canvas->selectTiles({0, 1});
  ASSERT_TRUE(process_until([legend]() {return legend->topLevelItemCount() > 0;}))
    << "cloud load never completed";

  EXPECT_TRUE(points->isEnabled())
    << "the selection took the colour control away instead of offering Pass";
  EXPECT_TRUE(comboRowEnabled(*points, pass_row));
  EXPECT_EQ(points->currentText(), "Pass") << "a new multi-pass selection defaults to Pass";

  // Deselectable: a scalar ramp is one click away, and it sticks.
  points->setCurrentIndex(comboRowOf(*points, "Backscatter"));
  EXPECT_EQ(points->currentText(), "Backscatter");
  EXPECT_TRUE(points->isEnabled());
  points->setCurrentIndex(pass_row);
  EXPECT_EQ(points->currentText(), "Pass");

  // Handing the pane back to the scrub window retires pass identity: Pass
  // greys out again and the colouring falls back rather than claiming a mode
  // it is not drawing.
  canvas->clearTileSelection();
  QCoreApplication::processEvents();
  EXPECT_FALSE(comboRowEnabled(*points, pass_row));
  EXPECT_EQ(points->currentText(), "Depth");
  EXPECT_TRUE(points->isEnabled());
}

// --- bounded teardown while a background job runs (#44) ---------------------

// The explorer's four heavy workers all read bags, so the only honest way to
// start a long one in a test is to give it a real bag. This writes a small
// synthetic recording whose window read is deliberately expensive — dense
// /tf plus many multi-beam detections — so an uncancelled cloud load takes
// long enough for "the close returned promptly" to mean something.
struct SyntheticBagSpec
{
  std::int64_t t0_ns = 1700000000000000000LL;
  std::int64_t t1_ns = 0;      // filled in by writeSyntheticBag
  int tf_messages = 12000;     // dynamic /tf samples across the window
  int pings = 4000;            // detection pings across the window
  int beams = 512;             // beams per ping
};

void writeSyntheticBag(const std::string & uri, SyntheticBagSpec & spec)
{
  constexpr std::int64_t kSpanNs = 60000000000LL;   // 60 s of recording
  spec.t1_ns = spec.t0_ns + kSpanNs;

  rosbag2_cpp::Writer writer;
  writer.open(uri);

  // Static mount: the sensor frame the detections are stamped in.
  {
    tf2_msgs::msg::TFMessage tfm;
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "bizzy/base_link";
    t.child_frame_id = "bizzy/m3";
    t.transform.translation.z = -0.5;
    t.transform.rotation.w = 1.0;
    t.header.stamp.sec = static_cast<std::int32_t>(spec.t0_ns / 1000000000LL);
    tfm.transforms.push_back(t);
    writer.write(tfm, "/tf_static", rclcpp::Time(spec.t0_ns));
  }

  // Dynamic chain: the boat under the map frame, and the earth<-map geo
  // anchor read_mbes_window looks up for cross-bag reprojection.
  for (int i = 0; i < spec.tf_messages; ++i) {
    const std::int64_t t =
      spec.t0_ns + kSpanNs * i / std::max(1, spec.tf_messages);
    tf2_msgs::msg::TFMessage tfm;
    geometry_msgs::msg::TransformStamped pose;
    pose.header.frame_id = "bizzy/map";
    pose.child_frame_id = "bizzy/base_link";
    pose.header.stamp.sec = static_cast<std::int32_t>(t / 1000000000LL);
    pose.header.stamp.nanosec = static_cast<std::uint32_t>(t % 1000000000LL);
    pose.transform.translation.x = 0.5 * i;
    pose.transform.rotation.w = 1.0;
    tfm.transforms.push_back(pose);

    geometry_msgs::msg::TransformStamped anchor = pose;
    anchor.header.frame_id = "earth";
    anchor.child_frame_id = "bizzy/map";
    anchor.transform.translation.x = 1917000.0;
    anchor.transform.translation.y = -4470000.0;
    anchor.transform.translation.z = 4325000.0;
    tfm.transforms.push_back(anchor);

    writer.write(tfm, "/tf", rclcpp::Time(t));
  }

  // Detections: every beam a real bottom detection, so the load does the
  // projection and the world-frame lift for each one.
  marine_acoustic_msgs::msg::SonarDetections det;
  det.header.frame_id = "bizzy/m3";
  det.ping_info.sound_speed = 1500.0f;
  det.two_way_travel_times.resize(static_cast<std::size_t>(spec.beams));
  det.tx_angles.assign(static_cast<std::size_t>(spec.beams), 0.0f);
  det.rx_angles.resize(static_cast<std::size_t>(spec.beams));
  det.intensities.assign(static_cast<std::size_t>(spec.beams), -30.0f);
  for (int b = 0; b < spec.beams; ++b) {
    const double angle = -1.0 + 2.0 * b / std::max(1, spec.beams - 1);
    det.rx_angles[static_cast<std::size_t>(b)] = static_cast<float>(angle);
    det.two_way_travel_times[static_cast<std::size_t>(b)] =
      static_cast<float>(2.0 * 12.0 / (1500.0 * std::cos(angle)));
  }
  for (int i = 0; i < spec.pings; ++i) {
    const std::int64_t t = spec.t0_ns + kSpanNs * i / std::max(1, spec.pings);
    det.header.stamp.sec = static_cast<std::int32_t>(t / 1000000000LL);
    det.header.stamp.nanosec = static_cast<std::uint32_t>(t % 1000000000LL);
    writer.write(det, "/bizzy/sensors/m3/detections", rclcpp::Time(t));
  }
}

// Closing the window while the cloud loader is reading bags must return the
// operator's prompt, not wait the load out (#44). The measurement is relative
// to the same load run to completion, so the assertion means the same thing
// on any machine: a teardown that took a meaningful fraction of the job it
// interrupted did not cancel it.
//
// This is also the granularity gate. The heavy pass is ONE pass over ONE bag,
// so a flag checked once per pass — or once before read_mbes_window's loops —
// bounds nothing here: the job runs to the end and the teardown takes as long
// as the baseline. Only a check inside the read loops passes.
TEST_F(ExplorerWindowFixture, ClosingDuringACloudLoadReturnsPromptly)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }

  const std::string bag_uri = std::string(::testing::TempDir()) + "/explorer_slow_bag";
  // ~35 MB of synthetic recording: erased on the way in and on every way
  // out, including a failed assertion (the temp dir is shared, and this is a
  // machine whose disk fills).
  struct BagCleanup
  {
    std::string uri;
    ~BagCleanup() {std::filesystem::remove_all(uri);}
  } cleanup{bag_uri};
  std::filesystem::remove_all(bag_uri);
  SyntheticBagSpec spec;
  writeSyntheticBag(bag_uri, spec);

  // Swap the fixture's missing-bag mbes pass for one over the real (slow)
  // bag, so the tile selection below drives exactly ONE long pass. That is
  // deliberate: with a second pass in the list, a flag checked only between
  // passes would bound the wait and this test would pass without ever
  // proving the check inside the bag read.
  sqlite3 * db = marine_survey_index::openIndexDb(db_path_);
  exec(db, "DELETE FROM passes WHERE sensor_type = 'mbes-bathy';");
  exec(db, ("INSERT INTO bags (id, path, size_bytes, mtime_ns, indexed_at_ns)"
    " VALUES (2, '" + bag_uri + "', 100, 200, 300);").c_str());
  const auto tile = gggs::Level(14).gridIndex(kLat, kLon);
  exec(db, ("INSERT INTO passes (bag_id, level, tile_row, tile_col, sensor_type,"
    " topic, t_start_ns, t_end_ns, ping_count) VALUES (2, 14, " +
    std::to_string(tile.row()) + ", " + std::to_string(tile.column()) +
    ", 'mbes-bathy', '/bizzy/sensors/m3/detections', " +
    std::to_string(spec.t0_ns) + ", " + std::to_string(spec.t1_ns) + ", " +
    std::to_string(spec.pings) + ");").c_str());
  sqlite3_close(db);

  qint64 baseline_ms = 0;
  {
    // Baseline: the same load, run to completion, so the bound below is
    // expressed against real work rather than a guessed millisecond count.
    auto window = std::make_unique<SidescanViewerWindow>();
    window->openSurveyIndex(db_path_, std::string(::testing::TempDir()));
    window->show();
    QCoreApplication::processEvents();
    auto * canvas = window->findChild<SidescanCanvas *>();
    ASSERT_NE(canvas, nullptr);
    auto * legend = window->findChild<QTreeWidget *>("cloud_legend");
    ASSERT_NE(legend, nullptr);

    QElapsedTimer timer;
    timer.start();
    canvas->selectTiles({0, 1});
    ASSERT_TRUE(
      process_until([legend]() {return legend->topLevelItemCount() > 0;}))
      << "the baseline cloud load never completed";
    baseline_ms = timer.elapsed();
  }
  // If the fixture stopped being a long job, the bound below would pass
  // vacuously. Fail loudly and say what to do instead of quietly asserting
  // nothing.
  ASSERT_GT(baseline_ms, 300)
    << "the synthetic bag is no longer slow enough for this test to mean "
       "anything (" << baseline_ms << " ms) — enlarge SyntheticBagSpec";

  // The real thing: start the same load and close mid-flight.
  auto window = std::make_unique<SidescanViewerWindow>();
  window->openSurveyIndex(db_path_, std::string(::testing::TempDir()));
  window->show();
  QCoreApplication::processEvents();
  auto * canvas = window->findChild<SidescanCanvas *>();
  ASSERT_NE(canvas, nullptr);
  canvas->selectTiles({0, 1});
  // Give the worker a head start so the close lands INSIDE the bag read.
  // Without it the cancel could arrive before the first message is read,
  // where even a once-per-pass check would look bounded — and the test would
  // stop being the granularity gate it is here to be. Spinning the event
  // loop, not sleeping: the worker is on a pool thread either way.
  QElapsedTimer head_start;
  head_start.start();
  while (head_start.elapsed() < 150) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
  }

  QElapsedTimer timer;
  timer.start();
  window->close();
  window.reset();   // ~SidescanViewerWindow waits on the workers
  const qint64 teardown_ms = timer.elapsed();

  EXPECT_LT(teardown_ms * 4, baseline_ms)
    << "closing took " << teardown_ms << " ms against a " << baseline_ms
    << " ms load: the cloud worker was waited out, not cancelled";
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

// --- metric measuring grid (#42) --------------------------------------------

// Render the canvas with the metric grid on or off. Like renderOverBox, the
// visibility toggle goes LAST because it invalidates the static layer cache.
QImage renderWithMetricGrid(SidescanCanvas & canvas, bool grid_on)
{
  canvas.fitGeo(kLat - 0.001, kLon - 0.001, kLat + 0.001, kLon + 0.001);
  canvas.setMetricGridVisible(grid_on);
  return canvas.grab().toImage();
}

// The grid must actually answer to its new switch: it is drawn into the
// cached static layer, so a toggle that forgot to invalidate the cache would
// leave the screen unchanged until something else forced a rebuild.
TEST_F(ExplorerWindowFixture, MetricGridTogglesOffAndBackOnScreen)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanCanvas canvas;
  canvas.resize(400, 300);
  canvas.setGeoOrigin(kLat, kLon);
  canvas.setGridSpacing(10.0);

  const QImage with_grid = renderWithMetricGrid(canvas, true);
  const QImage without_grid = renderWithMetricGrid(canvas, false);
  EXPECT_NE(with_grid, without_grid) << "hiding the metric grid changed nothing";

  // And back: the cache must invalidate in both directions.
  const QImage again = renderWithMetricGrid(canvas, true);
  EXPECT_EQ(again, with_grid) << "the metric grid did not come back";
}

// Mode-dependent default (#42): over a whole collection the grid measures
// from an arbitrary origin, so an open index starts with it off — while a
// bag, where the grid is the ruler a target is sized against, starts with it
// on. The spacing spinbox follows the grid it spaces.
TEST_F(ExplorerWindowFixture, MetricGridDefaultsOffInIndexModeAndOnForABag)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  // A previous run of this binary left its toggle in the redirected settings
  // store; the default under test is the no-preference one.
  {
    QSettings settings("UNH-CCOM", "survey_explorer");
    settings.remove("show_metric_grid");
    settings.sync();
  }

  SidescanViewerWindow bag_window;
  auto * bag_action = bag_window.findChild<QAction *>("show_metric_grid_action");
  ASSERT_NE(bag_action, nullptr);
  EXPECT_TRUE(bag_action->isChecked()) << "a bag lost its measuring grid";
  auto * bag_canvas = bag_window.findChild<SidescanCanvas *>();
  ASSERT_NE(bag_canvas, nullptr);
  EXPECT_TRUE(bag_canvas->metricGridVisible());

  SidescanViewerWindow window;
  window.openSurveyIndex(db_path_, std::string(::testing::TempDir()));
  window.show();
  QCoreApplication::processEvents();

  auto * action = window.findChild<QAction *>("show_metric_grid_action");
  ASSERT_NE(action, nullptr);
  // The View-menu action takes over the enable-on-index-open moment that used
  // to make the header checkbox visible (#42).
  EXPECT_TRUE(action->isEnabled());
  EXPECT_FALSE(action->isChecked()) << "the index map opened with the measuring grid on";
  auto * canvas = window.findChild<SidescanCanvas *>();
  ASSERT_NE(canvas, nullptr);
  EXPECT_FALSE(canvas->metricGridVisible());

  // The spacing spinbox spaces this grid, so it is greyed out with it.
  auto * spin = window.findChild<QDoubleSpinBox *>("grid_spacing_spin");
  ASSERT_NE(spin, nullptr);
  EXPECT_FALSE(spin->isEnabled());

  action->setChecked(true);
  QCoreApplication::processEvents();
  EXPECT_TRUE(canvas->metricGridVisible());
  EXPECT_TRUE(spin->isEnabled());
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

// --- region clearing: left-click vs. the context menu (#42) -----------------

void leftDrag(SidescanCanvas & canvas, const QPoint & from, const QPoint & to)
{
  QMouseEvent press(
    QEvent::MouseButtonPress, QPointF(from), QPointF(canvas.mapToGlobal(from)),
    Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(&canvas, &press);
  QMouseEvent move(
    QEvent::MouseMove, QPointF(to), QPointF(canvas.mapToGlobal(to)),
    Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(&canvas, &move);
  QMouseEvent release(
    QEvent::MouseButtonRelease, QPointF(to), QPointF(canvas.mapToGlobal(to)),
    Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
  QApplication::sendEvent(&canvas, &release);
}

// Press and release at the same point: a click, well inside the slop.
void leftClick(SidescanCanvas & canvas, const QPoint & pos)
{
  leftDrag(canvas, pos, pos);
}

// A fitted canvas whose index grid holds one tile around the fixture
// position, dragged over so a region exists to be cleared.
void selectARegion(SidescanCanvas & canvas)
{
  prepareFittedCanvas(canvas);
  canvas.setIndexTiles({GeoRect{kLat - 0.002, kLon - 0.002, kLat + 0.002, kLon + 0.002}});
  leftDrag(canvas, QPoint(40, 40), QPoint(360, 260));
  ASSERT_TRUE(canvas.hasRegion()) << "the drag never produced a region to clear";
}

QAction * findMenuAction(QMenu & menu, const QString & text)
{
  for (QAction * action : menu.actions()) {
    if (action->text() == text) {
      return action;
    }
  }
  return nullptr;
}

// The regression the operator hit: he clicked the map meaning to pick out a
// nav-track line and the click threw his region away. A bare left click must
// now leave the region exactly as it was, and must report nothing.
TEST_F(ExplorerWindowFixture, ALeftClickNoLongerClearsTheRegion)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanCanvas canvas;
  ASSERT_NO_FATAL_FAILURE(selectARegion(canvas));
  const std::set<std::size_t> selected_before = canvas.selectedTiles();
  ASSERT_FALSE(selected_before.empty());

  int cleared = 0;
  int selection_changed = 0;
  QObject::connect(
    &canvas, &SidescanCanvas::cubeBoxCleared, &canvas, [&cleared]() {++cleared;});
  QObject::connect(
    &canvas, &SidescanCanvas::tileSelectionChanged, &canvas,
    [&selection_changed]() {++selection_changed;});

  leftClick(canvas, QPoint(200, 150));

  EXPECT_TRUE(canvas.hasRegion());
  EXPECT_EQ(canvas.selectedTiles(), selected_before);
  EXPECT_EQ(cleared, 0);
  EXPECT_EQ(selection_changed, 0);
}

// Clearing is still available — it just has to be asked for by name.
TEST_F(ExplorerWindowFixture, TheContextMenuClearSelectionEntryClearsTheRegion)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanCanvas canvas;
  ASSERT_NO_FATAL_FAILURE(selectARegion(canvas));

  int cleared = 0;
  int selection_changed = 0;
  QObject::connect(
    &canvas, &SidescanCanvas::cubeBoxCleared, &canvas, [&cleared]() {++cleared;});
  QObject::connect(
    &canvas, &SidescanCanvas::tileSelectionChanged, &canvas,
    [&selection_changed]() {++selection_changed;});

  QMenu menu;
  std::unique_ptr<QMenu> built(canvas.buildContextMenu(&menu));
  QAction * clear = findMenuAction(*built, "Clear Selection");
  ASSERT_NE(clear, nullptr) << "the map menu offers no Clear Selection entry";
  EXPECT_TRUE(clear->isEnabled()) << "the entry was greyed out with a region selected";

  clear->trigger();

  EXPECT_FALSE(canvas.hasRegion());
  EXPECT_TRUE(canvas.selectedTiles().empty());
  EXPECT_EQ(cleared, 1);
  EXPECT_EQ(selection_changed, 1);
}

// With nothing selected the entry is still there (the menu keeps one shape)
// but greyed out, so it never reads as an action that would do something.
TEST_F(ExplorerWindowFixture, ClearSelectionIsDisabledWithNothingSelected)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanCanvas canvas;
  prepareFittedCanvas(canvas);
  ASSERT_FALSE(canvas.hasRegion());

  QMenu menu;
  std::unique_ptr<QMenu> built(canvas.buildContextMenu(&menu));
  QAction * clear = findMenuAction(*built, "Clear Selection");
  ASSERT_NE(clear, nullptr);
  EXPECT_FALSE(clear->isEnabled());
}

// The menu grows by adding an entry: a registered entry is offered in order,
// takes its enabled state from its own predicate, and runs on trigger.
TEST_F(ExplorerWindowFixture, RegisteredContextMenuEntriesAreOffered)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanCanvas canvas;
  prepareFittedCanvas(canvas);
  int runs = 0;
  bool applies = false;
  canvas.addContextMenuEntry(
    SidescanCanvas::ContextMenuEntry{
      "Test Entry", [&runs]() {++runs;}, [&applies]() {return applies;}});

  QMenu menu;
  {
    std::unique_ptr<QMenu> built(canvas.buildContextMenu(&menu));
    QAction * entry = findMenuAction(*built, "Test Entry");
    ASSERT_NE(entry, nullptr);
    EXPECT_FALSE(entry->isEnabled());
  }
  applies = true;
  std::unique_ptr<QMenu> built(canvas.buildContextMenu(&menu));
  QAction * entry = findMenuAction(*built, "Test Entry");
  ASSERT_NE(entry, nullptr);
  EXPECT_TRUE(entry->isEnabled());
  entry->trigger();
  EXPECT_EQ(runs, 1);
}

// --- CUBE uncertainty budget: presets seed, editing goes custom (#45) --------

// The order dropdown is a preset that seeds the two thresholds the run reads,
// so the window's tuning must move with it — and "custom" must never seed
// anything, because it is only ever arrived at by editing a threshold.
TEST_F(ExplorerWindowFixture, CubeOrderPresetsSeedTheUncertaintyBudget)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  auto * combo = window.findChild<QComboBox *>("cube_order_combo");
  ASSERT_NE(combo, nullptr);

  // Every preset is offered, plus custom, and nothing else.
  const auto presets = marine_perception_tools::iho_preset_names();
  ASSERT_EQ(combo->count(), static_cast<int>(presets.size()) + 1);
  for (std::size_t i = 0; i < presets.size(); ++i) {
    EXPECT_EQ(combo->itemText(static_cast<int>(i)).toStdString(), presets[i]);
  }
  EXPECT_EQ(
    combo->itemText(combo->count() - 1).toStdString(),
    std::string(marine_perception_tools::kCustomIhoOrder));
  // The order1a/order1b no-op pair is gone: no two entries are one budget.
  EXPECT_EQ(combo->findText("order1b"), -1);

  // It opens on whichever preset the library's own defaults are.
  const auto defaults = marine_perception_tools::default_cube_tuning();
  EXPECT_EQ(
    combo->currentText().toStdString(),
    marine_perception_tools::iho_order_for_limits(
      defaults.iho_fixed, defaults.iho_percent));

  // Picking a preset restores that preset's pair, in the tuning the next run
  // reads — not just in the label.
  for (const auto & name : presets) {
    const auto limits = marine_perception_tools::iho_preset_limits(name);
    ASSERT_TRUE(limits.has_value());
    combo->setCurrentText(QString::fromStdString(name));
    QCoreApplication::processEvents();
    EXPECT_FLOAT_EQ(window.cubeTuning().iho_fixed, limits->first) << name;
    EXPECT_FLOAT_EQ(window.cubeTuning().iho_percent, limits->second) << name;
  }

  // Selecting custom itself seeds nothing: the thresholds stay where the last
  // preset (or the operator's edit) left them.
  const float held_fixed = window.cubeTuning().iho_fixed;
  const float held_percent = window.cubeTuning().iho_percent;
  combo->setCurrentText(marine_perception_tools::kCustomIhoOrder);
  QCoreApplication::processEvents();
  EXPECT_FLOAT_EQ(window.cubeTuning().iho_fixed, held_fixed);
  EXPECT_FLOAT_EQ(window.cubeTuning().iho_percent, held_percent);
}

// --- a CUBE run adds a surface over the selection cloud (#36) ---------------

// A light synthetic recording centred on the fixture position: enough beams
// for CUBE to settle nodes, small enough that a selection load AND a run both
// finish inside a test. Unlike writeSyntheticBag (whose job is to be SLOW),
// the earth<-map anchor here is the fixture position itself, so the region
// drawn on the map really does clip to these soundings.
void writeLightBag(const std::string & uri, std::int64_t t0_ns, std::int64_t t1_ns)
{
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
  marine_perception_tools::geodetic_to_ecef(kLat, kLon, 0.0, ax, ay, az);
  const std::int64_t span = t1_ns - t0_ns;

  rosbag2_cpp::Writer writer;
  writer.open(uri);
  {
    tf2_msgs::msg::TFMessage tfm;
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "bizzy/base_link";
    t.child_frame_id = "bizzy/m3";
    t.transform.translation.z = -0.5;
    t.transform.rotation.w = 1.0;
    t.header.stamp.sec = static_cast<std::int32_t>(t0_ns / 1000000000LL);
    tfm.transforms.push_back(t);
    writer.write(tfm, "/tf_static", rclcpp::Time(t0_ns));
  }
  constexpr int kTfSamples = 200;
  for (int i = 0; i < kTfSamples; ++i) {
    const std::int64_t t = t0_ns + span * i / kTfSamples;
    tf2_msgs::msg::TFMessage tfm;
    geometry_msgs::msg::TransformStamped pose;
    pose.header.frame_id = "bizzy/map";
    pose.child_frame_id = "bizzy/base_link";
    pose.header.stamp.sec = static_cast<std::int32_t>(t / 1000000000LL);
    pose.header.stamp.nanosec = static_cast<std::uint32_t>(t % 1000000000LL);
    pose.transform.translation.x = 0.1 * i;   // ~20 m of line, well inside a region
    pose.transform.rotation.w = 1.0;
    tfm.transforms.push_back(pose);

    geometry_msgs::msg::TransformStamped anchor = pose;
    anchor.header.frame_id = "earth";
    anchor.child_frame_id = "bizzy/map";
    anchor.transform.translation.x = ax;
    anchor.transform.translation.y = ay;
    anchor.transform.translation.z = az;
    tfm.transforms.push_back(anchor);
    writer.write(tfm, "/tf", rclcpp::Time(t));
  }

  constexpr int kBeams = 64;
  constexpr int kPings = 80;
  marine_acoustic_msgs::msg::SonarDetections det;
  det.header.frame_id = "bizzy/m3";
  det.ping_info.sound_speed = 1500.0f;
  det.two_way_travel_times.resize(kBeams);
  det.tx_angles.assign(kBeams, 0.0f);
  det.rx_angles.resize(kBeams);
  det.intensities.assign(kBeams, -30.0f);
  for (int b = 0; b < kBeams; ++b) {
    const double angle = -0.6 + 1.2 * b / (kBeams - 1);
    det.rx_angles[static_cast<std::size_t>(b)] = static_cast<float>(angle);
    det.two_way_travel_times[static_cast<std::size_t>(b)] =
      static_cast<float>(2.0 * 12.0 / (1500.0 * std::cos(angle)));
  }
  for (int i = 0; i < kPings; ++i) {
    const std::int64_t t = t0_ns + span * i / kPings;
    det.header.stamp.sec = static_cast<std::int32_t>(t / 1000000000LL);
    det.header.stamp.nanosec = static_cast<std::uint32_t>(t % 1000000000LL);
    writer.write(det, "/bizzy/sensors/m3/detections", rclcpp::Time(t));
  }
}

// The operator's second complaint, as a test: a CUBE run must ADD a surface
// over the soundings already loaded, not replace them. Before this, a finished
// run swapped the multi-pass, pass-coloured cloud for its own separately
// gathered soundings — which read, rightly, as the run stealing the selection.
//
// The region is drawn once, with the one left-drag gesture that carries both
// consequences (#42): the tiles it covers load the cloud, its exact bounds are
// the processing extent. What must survive the run is everything that belongs
// to the selection — the per-pass cloud, its legend, and its colouring.
TEST_F(ExplorerWindowFixture, ACubeRunAddsASurfaceOverTheSelectionCloud)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }

  const std::string bag_uri = std::string(::testing::TempDir()) + "/explorer_cube_bag";
  struct BagCleanup
  {
    std::string uri;
    ~BagCleanup() {std::filesystem::remove_all(uri);}
  } cleanup{bag_uri};
  std::filesystem::remove_all(bag_uri);
  constexpr std::int64_t kT0 = 1700000000000000000LL;
  constexpr std::int64_t kT1 = kT0 + 20000000000LL;
  writeLightBag(bag_uri, kT0, kT1);

  // Swap the fixture's missing-bag pass for one over the real recording.
  sqlite3 * db = marine_survey_index::openIndexDb(db_path_);
  exec(db, "DELETE FROM passes WHERE sensor_type = 'mbes-bathy';");
  exec(db, ("INSERT INTO bags (id, path, size_bytes, mtime_ns, indexed_at_ns)"
    " VALUES (2, '" + bag_uri + "', 100, 200, 300);").c_str());
  const auto tile = gggs::Level(14).gridIndex(kLat, kLon);
  exec(db, ("INSERT INTO passes (bag_id, level, tile_row, tile_col, sensor_type,"
    " topic, t_start_ns, t_end_ns, ping_count) VALUES (2, 14, " +
    std::to_string(tile.row()) + ", " + std::to_string(tile.column()) +
    ", 'mbes-bathy', '/bizzy/sensors/m3/detections', " +
    std::to_string(kT0) + ", " + std::to_string(kT1) + ", 80);").c_str());
  sqlite3_close(db);

  SidescanViewerWindow window;
  window.resize(1200, 800);
  window.openSurveyIndex(db_path_, std::string(::testing::TempDir()));
  window.show();
  QCoreApplication::processEvents();

  auto * canvas = window.findChild<SidescanCanvas *>();
  auto * cloud = window.findChild<PointCloudView *>();
  auto * legend = window.findChild<QTreeWidget *>("cloud_legend");
  auto * points = window.findChild<QComboBox *>("cloud_color_combo");
  auto * run = window.findChild<QPushButton *>("cube_run_btn");
  auto * cell = window.findChild<QDoubleSpinBox *>("cube_cell_spin");
  ASSERT_NE(canvas, nullptr);
  ASSERT_NE(cloud, nullptr);
  ASSERT_NE(legend, nullptr);
  ASSERT_NE(points, nullptr);
  ASSERT_NE(run, nullptr);
  ASSERT_NE(cell, nullptr);
  canvas->grab();   // force the pending fit, so the drag reads a settled view

  // One region: the whole visible map, so it covers both the index tile and
  // the recorded line whatever the fit margin turned out to be.
  leftDrag(*canvas, QPoint(4, 4), QPoint(canvas->width() - 4, canvas->height() - 4));
  ASSERT_TRUE(
    process_until(
      [legend]() {
        return legend->topLevelItemCount() > 0 &&
               legend->topLevelItem(0)->text(1).toInt() > 0;
      }))
    << "the selection never loaded soundings from the synthetic bag";
  const int rows_before = legend->topLevelItemCount();
  ASSERT_EQ(cloud->passCount(), 1);
  ASSERT_EQ(points->currentText(), "Pass");
  ASSERT_FALSE(cloud->hasSurface());

  // Coarse cells: the surface only has to exist, and a 0.1 m grid over a
  // whole map view is a lot of nodes to settle inside a test.
  cell->setValue(2.0);
  ASSERT_TRUE(run->isEnabled()) << "the region did not arm the CUBE run";
  run->click();
  ASSERT_TRUE(process_until([cloud]() {return cloud->hasSurface();}))
    << "the CUBE run never produced a surface";

  // The selection is still what the operator is looking at — now with a
  // surface over it.
  EXPECT_EQ(cloud->passCount(), 1) << "the run replaced the multi-pass cloud";
  EXPECT_TRUE(legend->isVisible());
  EXPECT_EQ(legend->topLevelItemCount(), rows_before);
  EXPECT_EQ(points->currentText(), "Pass")
    << "the run re-coloured the operator's cloud by depth";
  EXPECT_TRUE(points->isEnabled());
}

// --- the nav-track fix under the cursor (#46) --------------------------------

// The inverse of pixelGeo: where a geographic point lands on screen. Same
// linear equirectangular mapping, read off the settled visible region.
QPoint geoPixel(const SidescanCanvas & canvas, double lat, double lon)
{
  const GeoRect r = canvas.visibleGeoRegion().value();
  return QPoint(
    static_cast<int>(std::lround((lon - r.west) / (r.east - r.west) * canvas.width())),
    static_cast<int>(std::lround((r.north - lat) / (r.north - r.south) * canvas.height())));
}

void hoverAt(SidescanCanvas & canvas, const QPoint & pos)
{
  QMouseEvent move(
    QEvent::MouseMove, QPointF(pos), QPointF(canvas.mapToGlobal(pos)),
    Qt::NoButton, Qt::NoButton, Qt::NoModifier);
  QApplication::sendEvent(&canvas, &move);
}

// A shown explorer window whose map is fitted tightly around the fixture's
// nav track, so a screen pixel is a fraction of a metre and the two fixes
// (11 m apart) are tens of pixels apart — the survey zoom the feature is used
// at, rather than a campaign-wide view where everything is within the radius.
SidescanCanvas * prepareHoverWindow(
  SidescanViewerWindow & window, const std::string & db_path)
{
  window.openSurveyIndex(db_path, std::string(::testing::TempDir()));
  window.show();
  QCoreApplication::processEvents();
  auto * canvas = window.findChild<SidescanCanvas *>();
  if (!canvas) {
    return nullptr;
  }
  canvas->fitGeo(kLat - 0.0005, kLon - 0.001, kLat + 0.0015, kLon + 0.001);
  canvas->grab();   // the fit only lands on a laid-out paint
  return canvas;
}

// The question the map can answer and the time bar cannot: when did THAT pass
// happen. Hovering near the drawn track marks the nearest fix and reads out
// its time; moving away takes both back.
TEST_F(ExplorerWindowFixture, HoveringNearTheTrackHighlightsTheNearestFixAndItsTime)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  SidescanCanvas * canvas = prepareHoverWindow(window, db_path_);
  ASSERT_NE(canvas, nullptr);
  auto * readout = window.findChild<QLabel *>("hover_time");
  ASSERT_NE(readout, nullptr);
  auto * timeline = window.findChild<TimeBarWidget *>("time_bar");
  ASSERT_NE(timeline, nullptr);
  EXPECT_TRUE(readout->text().isEmpty()) << "a readout with nothing hovered";
  EXPECT_FALSE(canvas->highlightedFix().has_value());

  // Straight onto the SECOND fixture fix (t = 2000 ns).
  hoverAt(*canvas, geoPixel(*canvas, kLat + 1e-4, kLon));

  ASSERT_TRUE(canvas->highlightedFix().has_value()) << "nothing highlighted on the track";
  EXPECT_NEAR(canvas->highlightedFix()->lat, kLat + 1e-4, 1e-9);
  EXPECT_NEAR(canvas->highlightedFix()->lon, kLon, 1e-9);
  // The time bar's own formatter, so one instant never reads two ways.
  EXPECT_EQ(readout->text(), timeline->formatTime(2000));

  // The other fix is the near one from a pixel beside it.
  hoverAt(*canvas, geoPixel(*canvas, kLat, kLon));
  ASSERT_TRUE(canvas->highlightedFix().has_value());
  EXPECT_NEAR(canvas->highlightedFix()->lat, kLat, 1e-9);
  EXPECT_EQ(readout->text(), timeline->formatTime(1000));

  // Off the track entirely: no marker, no readout. The feature is silent
  // whenever the operator is not asking it anything.
  hoverAt(*canvas, QPoint(3, canvas->height() / 2));
  EXPECT_FALSE(canvas->highlightedFix().has_value());
  EXPECT_TRUE(readout->text().isEmpty());
}

// The readout is a time display like every other one in the window, so it
// follows the one UTC/local toggle rather than answering in its own zone.
TEST_F(ExplorerWindowFixture, TheHoveredFixTimeFollowsTheUtcToggle)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  SidescanCanvas * canvas = prepareHoverWindow(window, db_path_);
  ASSERT_NE(canvas, nullptr);
  auto * readout = window.findChild<QLabel *>("hover_time");
  auto * utc = window.findChild<QCheckBox *>("utc_check");
  auto * timeline = window.findChild<TimeBarWidget *>("time_bar");
  ASSERT_NE(readout, nullptr);
  ASSERT_NE(utc, nullptr);
  ASSERT_NE(timeline, nullptr);
  ASSERT_FALSE(utc->isChecked()) << "local time is the default (#26)";

  hoverAt(*canvas, geoPixel(*canvas, kLat, kLon));
  ASSERT_FALSE(readout->text().isEmpty());
  EXPECT_EQ(readout->text(), timeline->formatTime(1000));

  // Switching zone re-renders what is already on screen — the operator must
  // not have to jiggle the mouse to get the answer he just asked for.
  utc->setChecked(true);
  EXPECT_TRUE(readout->text().endsWith(" UTC"));
  EXPECT_EQ(readout->text(), timeline->formatTime(1000));
}

// The point of the whole feature: the click cues. It goes through the same
// path a committed time on the time bar takes, so there is one cueing
// implementation and the map and the tape cannot disagree.
TEST_F(ExplorerWindowFixture, ClickingAHighlightedFixCuesThatInstant)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  SidescanCanvas * canvas = prepareHoverWindow(window, db_path_);
  ASSERT_NE(canvas, nullptr);
  auto * status = window.findChild<QLabel *>("status");
  ASSERT_NE(status, nullptr);

  hoverAt(*canvas, geoPixel(*canvas, kLat, kLon));
  ASSERT_TRUE(canvas->highlightedFix().has_value());

  leftClick(*canvas, geoPixel(*canvas, kLat, kLon));

  // The instant is in the fixture's one recording, which is not open, so the
  // cue resolves to that bag — and says so. The reopen is the known, and
  // deliberately out-of-scope, cost of cueing across recordings (#36); what
  // this feature owes the operator is that he can tell it is happening.
  EXPECT_TRUE(status->text().contains("Cueing")) << status->text().toStdString();
  EXPECT_TRUE(status->text().contains("bag_a")) << status->text().toStdString();
  EXPECT_TRUE(status->text().contains("another recording")) << status->text().toStdString();
}

// #42 made a bare left click do nothing, because the one thing it used to do
// was silently destroy the operator's region. That stands: this issue adds an
// action on a HIT only, and off the track the click is still a no-op.
TEST_F(ExplorerWindowFixture, AClickWithNoHighlightedFixStillDoesNothing)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  SidescanCanvas * canvas = prepareHoverWindow(window, db_path_);
  ASSERT_NE(canvas, nullptr);
  auto * status = window.findChild<QLabel *>("status");
  ASSERT_NE(status, nullptr);

  // Draw a region, then step off the track so nothing is highlighted.
  leftDrag(*canvas, QPoint(20, 20), QPoint(canvas->width() - 20, canvas->height() - 20));
  ASSERT_TRUE(canvas->hasRegion()) << "the drag never produced a region";
  const std::set<std::size_t> selected_before = canvas->selectedTiles();
  const QPoint away(3, canvas->height() / 2);
  hoverAt(*canvas, away);
  ASSERT_FALSE(canvas->highlightedFix().has_value());
  const QString status_before = status->text();

  int cleared = 0;
  int selection_changed = 0;
  QObject::connect(
    canvas, &SidescanCanvas::cubeBoxCleared, canvas, [&cleared]() {++cleared;});
  QObject::connect(
    canvas, &SidescanCanvas::tileSelectionChanged, canvas,
    [&selection_changed]() {++selection_changed;});

  leftClick(*canvas, away);

  EXPECT_TRUE(canvas->hasRegion());
  EXPECT_EQ(canvas->selectedTiles(), selected_before);
  EXPECT_EQ(cleared, 0);
  EXPECT_EQ(selection_changed, 0);
  EXPECT_EQ(status->text(), status_before) << "a miss cued something";
}

// The highlight must not fight the gestures the map already has. A left drag
// draws the region; dragging it across the track must not light fixes up
// under the rubber band, and the release must draw a region rather than cue.
TEST_F(ExplorerWindowFixture, ARegionDragAcrossTheTrackNeverHighlights)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  SidescanCanvas * canvas = prepareHoverWindow(window, db_path_);
  ASSERT_NE(canvas, nullptr);
  auto * status = window.findChild<QLabel *>("status");
  auto * readout = window.findChild<QLabel *>("hover_time");
  ASSERT_NE(status, nullptr);
  ASSERT_NE(readout, nullptr);

  // Highlighted first, so the drag has something to take away.
  const QPoint on_fix = geoPixel(*canvas, kLat, kLon);
  hoverAt(*canvas, on_fix);
  ASSERT_TRUE(canvas->highlightedFix().has_value());

  const QPoint from(on_fix.x() - 40, on_fix.y() - 40);
  QMouseEvent press(
    QEvent::MouseButtonPress, QPointF(from), QPointF(canvas->mapToGlobal(from)),
    Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
  QApplication::sendEvent(canvas, &press);
  // The press alone is not yet a gesture — it is still on its way to being a
  // click — so it must not have cleared anything.
  EXPECT_TRUE(canvas->highlightedFix().has_value())
    << "the press cleared the highlight the release might have cued";

  // Now travel, straight over the fix.
  for (const QPoint & at : {QPoint(on_fix.x() - 10, on_fix.y() - 10), on_fix,
      QPoint(on_fix.x() + 40, on_fix.y() + 40)})
  {
    QMouseEvent move(
      QEvent::MouseMove, QPointF(at), QPointF(canvas->mapToGlobal(at)),
      Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(canvas, &move);
    EXPECT_TRUE(canvas->pointerGestureActive()) << "a travelled drag is a gesture";
    EXPECT_FALSE(canvas->highlightedFix().has_value())
      << "a fix lit up under the rubber band";
    EXPECT_TRUE(readout->text().isEmpty());
  }
  const QPoint to(on_fix.x() + 40, on_fix.y() + 40);
  QMouseEvent release(
    QEvent::MouseButtonRelease, QPointF(to), QPointF(canvas->mapToGlobal(to)),
    Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
  QApplication::sendEvent(canvas, &release);

  // The release reports the region it drew (its status line says so) and
  // cues nothing: a drag is not a click, however exactly it crossed the track.
  EXPECT_TRUE(canvas->hasRegion()) << "the drag drew no region";
  EXPECT_FALSE(status->text().contains("Cueing"))
    << "the drag cued as well as selecting: " << status->text().toStdString();
  EXPECT_FALSE(canvas->pointerGestureActive()) << "the settled drag is still a gesture";
}

// The other two view-moving gestures: a middle-drag pan, and the recentre
// glide a middle click starts. Both move the map out from under a pointer
// that is not itself moving, so a highlight left standing would be pointing
// at water the boat never crossed.
TEST_F(ExplorerWindowFixture, PanningAndTheRecentreGlideClearTheHighlight)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  SidescanCanvas * canvas = prepareHoverWindow(window, db_path_);
  ASSERT_NE(canvas, nullptr);
  auto * readout = window.findChild<QLabel *>("hover_time");
  ASSERT_NE(readout, nullptr);
  const QPoint on_fix = geoPixel(*canvas, kLat, kLon);

  // A middle-drag pan.
  hoverAt(*canvas, on_fix);
  ASSERT_TRUE(canvas->highlightedFix().has_value());
  QMouseEvent press(
    QEvent::MouseButtonPress, QPointF(on_fix), QPointF(canvas->mapToGlobal(on_fix)),
    Qt::MiddleButton, Qt::MiddleButton, Qt::NoModifier);
  QApplication::sendEvent(canvas, &press);
  const QPoint moved(on_fix.x() + 60, on_fix.y() + 20);
  QMouseEvent drag(
    QEvent::MouseMove, QPointF(moved), QPointF(canvas->mapToGlobal(moved)),
    Qt::NoButton, Qt::MiddleButton, Qt::NoModifier);
  QApplication::sendEvent(canvas, &drag);
  EXPECT_TRUE(canvas->pointerGestureActive());
  EXPECT_FALSE(canvas->highlightedFix().has_value()) << "the highlight rode the pan";
  EXPECT_TRUE(readout->text().isEmpty());
  QMouseEvent up(
    QEvent::MouseButtonRelease, QPointF(moved), QPointF(canvas->mapToGlobal(moved)),
    Qt::MiddleButton, Qt::NoButton, Qt::NoModifier);
  QApplication::sendEvent(canvas, &up);
  EXPECT_FALSE(canvas->pointerGestureActive()) << "the settled pan is still a gesture";

  // And the recentre glide: a middle CLICK, which travels nowhere.
  canvas->grab();   // settle the panned view before reading pixels off it
  const QPoint fix_now = geoPixel(*canvas, kLat, kLon);
  hoverAt(*canvas, fix_now);
  ASSERT_TRUE(canvas->highlightedFix().has_value());
  middleClick(*canvas, QPoint(canvas->width() - 10, 10));
  ASSERT_TRUE(canvas->recenterAnimating()) << "no glide to test against";
  EXPECT_TRUE(canvas->pointerGestureActive());
  EXPECT_FALSE(canvas->highlightedFix().has_value()) << "the highlight rode the glide";
  EXPECT_TRUE(readout->text().isEmpty());
  canvas->finishRecenterNow();
}

// A zoom moves the map under a pointer that did not move and changes what
// "close on screen" means, so no move event will ever come along to correct a
// stale highlight. It clears; the next move re-searches at the new scale.
TEST_F(ExplorerWindowFixture, AWheelZoomClearsTheHighlight)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  SidescanCanvas * canvas = prepareHoverWindow(window, db_path_);
  ASSERT_NE(canvas, nullptr);
  auto * readout = window.findChild<QLabel *>("hover_time");
  ASSERT_NE(readout, nullptr);

  const QPoint on_fix = geoPixel(*canvas, kLat, kLon);
  hoverAt(*canvas, on_fix);
  ASSERT_TRUE(canvas->highlightedFix().has_value());

  const QPointF centre_px(canvas->width() / 2.0, canvas->height() / 2.0);
  QWheelEvent wheel(
    centre_px, QPointF(canvas->mapToGlobal(centre_px.toPoint())), QPoint(0, 0),
    QPoint(0, 120), Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
  QApplication::sendEvent(canvas, &wheel);

  EXPECT_FALSE(canvas->highlightedFix().has_value());
  EXPECT_TRUE(readout->text().isEmpty());

  // And it comes back at the new scale on the next move.
  canvas->grab();
  hoverAt(*canvas, geoPixel(*canvas, kLat, kLon));
  EXPECT_TRUE(canvas->highlightedFix().has_value());
}

// --- the geographic cursor readout (#47) -------------------------------------

// The map's own conversion: the position under the cursor, to the six
// decimals the readout has always used, tagged with the pane it came from so
// four panes feeding one label stay tellable apart.
TEST_F(ExplorerWindowFixture, TheMapReadsOutTheCursorsPositionNamingThePane)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  SidescanCanvas * canvas = prepareHoverWindow(window, db_path_);
  ASSERT_NE(canvas, nullptr);
  auto * geo = window.findChild<QLabel *>("hover_geo");
  ASSERT_NE(geo, nullptr);
  EXPECT_TRUE(geo->text().isEmpty()) << "a position with nothing hovered";

  hoverAt(*canvas, geoPixel(*canvas, kLat, kLon));

  const QString shown = geo->text();
  ASSERT_TRUE(shown.startsWith("Map  ")) << shown.toStdString();
  const QStringList parts = shown.mid(5).split(", ");
  ASSERT_EQ(parts.size(), 2) << shown.toStdString();
  EXPECT_NEAR(parts[0].toDouble(), kLat, 1e-5);
  EXPECT_NEAR(parts[1].toDouble(), kLon, 1e-5);
  // Six decimals: the format the map readout has always used (~0.1 m).
  EXPECT_EQ(parts[0].section('.', 1).size(), 6) << shown.toStdString();
  EXPECT_EQ(parts[1].section('.', 1).size(), 6) << shown.toStdString();
}

// Nothing rather than zero: with no index and no bag there is no earth
// reference anywhere, so the map cannot place the cursor — and a readout of
// "0.000000, 0.000000" would be a position off West Africa.
TEST_F(ExplorerWindowFixture, AnUnplaceableMapReadsOutNothingRatherThanZero)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  window.show();
  QCoreApplication::processEvents();
  auto * canvas = window.findChild<SidescanCanvas *>();
  ASSERT_NE(canvas, nullptr);
  auto * geo = window.findChild<QLabel *>("hover_geo");
  ASSERT_NE(geo, nullptr);

  hoverAt(*canvas, QPoint(canvas->width() / 2, canvas->height() / 2));

  EXPECT_TRUE(geo->text().isEmpty()) << geo->text().toStdString();
}

// Hovering a pane that cannot place the cursor must BLANK the readout, not
// leave the map's last position standing where it reads as live. That stale
// number — indistinguishable from a live one — is the reason this exists.
TEST_F(ExplorerWindowFixture, HoveringAPaneThatCannotPlaceTheCursorBlanksTheReadout)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  SidescanViewerWindow window;
  SidescanCanvas * canvas = prepareHoverWindow(window, db_path_);
  ASSERT_NE(canvas, nullptr);
  auto * geo = window.findChild<QLabel *>("hover_geo");
  auto * waterfall =
    window.findChild<marine_sonar_widgets::WaterfallWidget *>("sidescan_waterfall");
  ASSERT_NE(geo, nullptr);
  ASSERT_NE(waterfall, nullptr);

  hoverAt(*canvas, geoPixel(*canvas, kLat, kLon));
  ASSERT_FALSE(geo->text().isEmpty()) << "the map never reported a position";

  // No bag is open, so the waterfall has no pose to project a pixel through.
  const QPoint p(waterfall->width() / 2, waterfall->height() / 2);
  QMouseEvent move(
    QEvent::MouseMove, QPointF(p), QPointF(waterfall->mapToGlobal(p)),
    Qt::NoButton, Qt::NoButton, Qt::NoModifier);
  QApplication::sendEvent(waterfall, &move);

  EXPECT_TRUE(geo->text().isEmpty()) << geo->text().toStdString();
}

// A small openable recording at the fixture position: the Garmin sidescan
// ping topics a session requires, MBES detections for the 3D pane and the
// backscatter waterfall, and the /tf chain (including the earth<-map anchor)
// that places all of it geographically.
void writeSidescanBag(const std::string & uri, std::int64_t t0_ns, std::int64_t t1_ns)
{
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
  marine_perception_tools::geodetic_to_ecef(kLat, kLon, 0.0, ax, ay, az);
  const std::int64_t span = t1_ns - t0_ns;

  rosbag2_cpp::Writer writer;
  writer.open(uri);
  {
    tf2_msgs::msg::TFMessage tfm;
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "bizzy/base_link";
    t.child_frame_id = "bizzy/m3";
    t.transform.translation.z = -0.5;
    t.transform.rotation.w = 1.0;
    t.header.stamp.sec = static_cast<std::int32_t>(t0_ns / 1000000000LL);
    tfm.transforms.push_back(t);
    writer.write(tfm, "/tf_static", rclcpp::Time(t0_ns));
  }
  constexpr int kTfSamples = 120;
  for (int i = 0; i <= kTfSamples; ++i) {
    const std::int64_t t = t0_ns + span * i / kTfSamples;
    tf2_msgs::msg::TFMessage tfm;
    geometry_msgs::msg::TransformStamped pose;
    pose.header.frame_id = "bizzy/map";
    pose.child_frame_id = "bizzy/base_link";
    pose.header.stamp.sec = static_cast<std::int32_t>(t / 1000000000LL);
    pose.header.stamp.nanosec = static_cast<std::uint32_t>(t % 1000000000LL);
    pose.transform.translation.x = 0.2 * i;   // ~24 m of line
    pose.transform.rotation.w = 1.0;
    tfm.transforms.push_back(pose);

    geometry_msgs::msg::TransformStamped anchor = pose;
    anchor.header.frame_id = "earth";
    anchor.child_frame_id = "bizzy/map";
    anchor.transform.translation.x = ax;
    anchor.transform.translation.y = ay;
    anchor.transform.translation.z = az;
    tfm.transforms.push_back(anchor);
    writer.write(tfm, "/tf", rclcpp::Time(t));
  }

  // Sidescan pings: one beam of amplitudes per channel per ping, bottom
  // return roughly a third of the way out so the row has structure.
  constexpr std::uint32_t kSamples = 256;
  constexpr int kPings = 120;
  marine_acoustic_msgs::msg::RawSonarImage img;
  img.header.frame_id = "bizzy/garmin_sidescan";
  img.ping_info.sound_speed = 1500.0f;
  img.sample_rate = 20000.0f;
  img.samples_per_beam = kSamples;
  img.sample0 = 0;
  img.image.dtype = marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8;
  img.image.beam_count = 1;
  img.image.data.resize(kSamples);
  for (std::uint32_t k = 0; k < kSamples; ++k) {
    img.image.data[k] = static_cast<std::uint8_t>(k > kSamples / 3 ? 200 : 20);
  }
  for (int i = 0; i < kPings; ++i) {
    const std::int64_t t = t0_ns + span * i / kPings;
    img.header.stamp.sec = static_cast<std::int32_t>(t / 1000000000LL);
    img.header.stamp.nanosec = static_cast<std::uint32_t>(t % 1000000000LL);
    writer.write(
      img, "/bizzy/sensors/sidescan/garmin_sidescan/sonar_image_port",
      rclcpp::Time(t));
    writer.write(
      img, "/bizzy/sensors/sidescan/garmin_sidescan/sonar_image_starboard",
      rclcpp::Time(t));
  }

  constexpr int kBeams = 32;
  marine_acoustic_msgs::msg::SonarDetections det;
  det.header.frame_id = "bizzy/m3";
  det.ping_info.sound_speed = 1500.0f;
  det.two_way_travel_times.resize(kBeams);
  det.tx_angles.assign(kBeams, 0.0f);
  det.rx_angles.resize(kBeams);
  det.intensities.assign(kBeams, -30.0f);
  for (int b = 0; b < kBeams; ++b) {
    const double angle = -0.6 + 1.2 * b / (kBeams - 1);
    det.rx_angles[static_cast<std::size_t>(b)] = static_cast<float>(angle);
    det.two_way_travel_times[static_cast<std::size_t>(b)] =
      static_cast<float>(2.0 * 12.0 / (1500.0 * std::cos(angle)));
  }
  for (int i = 0; i < kPings; ++i) {
    const std::int64_t t = t0_ns + span * i / kPings;
    det.header.stamp.sec = static_cast<std::int32_t>(t / 1000000000LL);
    det.header.stamp.nanosec = static_cast<std::uint32_t>(t % 1000000000LL);
    writer.write(det, "/bizzy/sensors/m3/detections", rclcpp::Time(t));
  }
}

// The three panes that hover in a bag's map-ENU frame, each converting at its
// own source: the 3D cloud through its load's frame, the MBES waterfall
// through the open bag's, and the echogram through the track position at the
// along-track distance under the cursor. All three read out the recording's
// own position, and say which pane said so.
TEST_F(ExplorerWindowFixture, EverySpatialPaneReadsOutItsOwnFramesPosition)
{
  app();
  if (!gl_available()) {
    GTEST_SKIP() << "no usable offscreen GL context";
  }
  const std::string bag_uri = std::string(::testing::TempDir()) + "/explorer_pane_geo_bag";
  struct BagCleanup
  {
    std::string uri;
    ~BagCleanup() {std::filesystem::remove_all(uri);}
  } cleanup{bag_uri};
  std::filesystem::remove_all(bag_uri);
  constexpr std::int64_t kT0 = 1700000000000000000LL;
  constexpr std::int64_t kT1 = kT0 + 20000000000LL;
  writeSidescanBag(bag_uri, kT0, kT1);

  SidescanViewerWindow window;
  window.setCacheDir(std::string(::testing::TempDir()) + "/pane_geo_cache");
  window.resize(1200, 800);
  window.openBag(bag_uri);
  window.show();

  auto * geo = window.findChild<QLabel *>("hover_geo");
  auto * cloud = window.findChild<PointCloudView *>("cloud_view");
  auto * waterfall =
    window.findChild<marine_sonar_widgets::WaterfallWidget *>("sidescan_waterfall");
  auto * echogram = window.findChild<marine_sonar_widgets::EchogramWidget *>("echogram");
  ASSERT_NE(geo, nullptr);
  ASSERT_NE(cloud, nullptr);
  ASSERT_NE(waterfall, nullptr);
  ASSERT_NE(echogram, nullptr);
  // The scrub window trails the head, so at distance 0 it is empty: drive to
  // the end of the line, where the window covers the recording.
  auto * scrub = window.findChild<QSlider *>();
  ASSERT_NE(scrub, nullptr);
  ASSERT_TRUE(process_until([scrub]() {return scrub->isEnabled() && scrub->maximum() > 1;}))
    << "the bag never opened";
  scrub->setValue(scrub->maximum());
  ASSERT_TRUE(process_until([cloud]() {return cloud->pointCount() > 0;}))
    << "the bag's soundings never reached the 3D pane";

  // Read a position out of a pane and check it is the recording's own, to the
  // ~20 m the synthetic line covers.
  const auto reads_position = [geo](const char * pane) {
      const QString shown = geo->text();
      EXPECT_TRUE(shown.startsWith(QString(pane) + "  ")) << shown.toStdString();
      const QStringList parts =
        shown.mid(static_cast<int>(std::strlen(pane)) + 2).split(", ");
      if (parts.size() != 2) {
        ADD_FAILURE() << "not a lat/lon: " << shown.toStdString();
        return;
      }
      EXPECT_NEAR(parts[0].toDouble(), kLat, 1e-3);
      EXPECT_NEAR(parts[1].toDouble(), kLon, 1e-3);
    };
  const auto hover_widget = [](QWidget * w) {
      const QPoint p(w->width() / 2, w->height() / 2);
      QMouseEvent move(
        QEvent::MouseMove, QPointF(p), QPointF(w->mapToGlobal(p)),
        Qt::NoButton, Qt::NoButton, Qt::NoModifier);
      QApplication::sendEvent(w, &move);
    };

  // The 3D view's ground-plane pick needs a painted camera to unproject through.
  cloud->grabFramebuffer();
  hover_widget(cloud);
  reads_position("MBES 3D");

  hover_widget(waterfall);
  reads_position("Sidescan");

  auto * mbes =
    window.findChild<marine_sonar_widgets::WaterfallWidget *>("mbes_waterfall");
  ASSERT_NE(mbes, nullptr);
  hover_widget(mbes);
  reads_position("MBES Backscatter");

  hover_widget(echogram);
  reads_position("Water Column");
}

}  // namespace
