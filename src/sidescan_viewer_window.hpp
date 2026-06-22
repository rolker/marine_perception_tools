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

#ifndef SIDESCAN_VIEWER_WINDOW_HPP_
#define SIDESCAN_VIEWER_WINDOW_HPP_

#include <QFutureWatcher>
#include <QImage>
#include <QMainWindow>
#include <QString>

#include <cstddef>
#include <memory>
#include <string>

#include "sidescan_bag_session.hpp"

class QLabel;
class QSlider;
class QDoubleSpinBox;
class QProgressBar;

namespace marine_perception_tools
{

class SidescanCanvas;
class SidescanWaterfall;

// Result of an off-thread bag load: either a session or an error message. Copyable
// (shared_ptr + QString) so it can ride through QFuture/QFutureWatcher.
struct SidescanLoadResult
{
  std::shared_ptr<SidescanBagSession> session;
  QString error;
};

// Result of an off-thread window render: the painted coverage image + its map
// extent. Built on a worker thread (readWindow + paint + rasterize) so scrubbing
// never blocks the UI; only the final setCoverage runs on the UI thread.
struct SidescanRenderResult
{
  bool ok = false;
  QImage image;          // georeferenced coverage (map frame)
  QImage waterfall;      // uncorrected slant-range waterfall of the same window
  double origin_x = 0.0;
  double origin_y = 0.0;
  double res_m = 0.25;
  double center_x = 0.0;  // map centre of the window's painted swath
  double center_y = 0.0;
  bool has_center = false;
  std::size_t npings = 0;
  double head_m = 0.0;
  double total_m = 0.0;
  double win_lo = 0.0;
  double win_hi = 0.0;
};

// Offline sidescan viewer main window: File->Open a bag, then scrub along
// distance travelled. A rolling ~window of pings is painted (quality-wins) into a
// coverage raster at true map position and shown north-up on the canvas, with the
// boat track and a measuring grid. PR2 scope: the geo-map pane + scrub; the target
// list / echogram panes come in later PRs.
class SidescanViewerWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit SidescanViewerWindow(QWidget * parent = nullptr);

  // Open a bag directly (e.g. from a CLI argument).
  void openBag(const std::string & bag_uri);

private slots:
  void onOpenBag();
  void onLoadFinished();
  void onRenderFinished();
  void onScrubChanged();
  void onGridSpacingChanged(double metres);
  void onWindowLengthChanged(double metres);

private:
  // Launch a window render on a worker thread, coalescing rapid scrub changes:
  // if a render is in flight, just flag a pending one and re-launch on finish
  // with the latest scrub position.
  void requestRender();

  SidescanCanvas * canvas_ = nullptr;
  QSlider * scrub_ = nullptr;
  QDoubleSpinBox * grid_spin_ = nullptr;
  QDoubleSpinBox * window_spin_ = nullptr;
  QLabel * status_ = nullptr;
  QProgressBar * progress_ = nullptr;
  SidescanWaterfall * waterfall_ = nullptr;

  QFutureWatcher<SidescanLoadResult> load_watcher_;
  QFutureWatcher<SidescanRenderResult> render_watcher_;
  bool loading_ = false;
  bool rendering_ = false;
  bool render_pending_ = false;

  std::shared_ptr<SidescanBagSession> session_;
  double window_len_m_ = 100.0;
  double resolution_m_ = 0.25;
  int max_window_pings_ = 600;   // stationary cap
};

}  // namespace marine_perception_tools

#endif  // SIDESCAN_VIEWER_WINDOW_HPP_
