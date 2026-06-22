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

#include <QMainWindow>

#include <memory>
#include <string>

#include "sidescan_bag_session.hpp"

class QLabel;
class QSlider;
class QDoubleSpinBox;

namespace marine_perception_tools
{

class SidescanCanvas;

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
  void onScrubChanged();
  void onGridSpacingChanged(double metres);
  void onWindowLengthChanged(double metres);

private:
  void renderCurrentWindow();

  SidescanCanvas * canvas_ = nullptr;
  QSlider * scrub_ = nullptr;
  QDoubleSpinBox * grid_spin_ = nullptr;
  QDoubleSpinBox * window_spin_ = nullptr;
  QLabel * status_ = nullptr;

  std::unique_ptr<SidescanBagSession> session_;
  double window_len_m_ = 100.0;
  double resolution_m_ = 0.25;
  int max_window_pings_ = 600;   // stationary cap
};

}  // namespace marine_perception_tools

#endif  // SIDESCAN_VIEWER_WINDOW_HPP_
