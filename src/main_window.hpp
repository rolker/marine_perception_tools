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

#ifndef MAIN_WINDOW_HPP_
#define MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QString>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "bag_loader.hpp"
#include "resim_engine.hpp"

class QDoubleSpinBox;
class QLabel;
class QSlider;

namespace marine_perception_tools
{

// The sea_surface_tuner main window: a forward-camera segmentation pane beside
// the re-simulated costmap, a frame scrubber, and a parameter dock. Editing a
// knob re-simulates and refreshes the costmap; scrubbing seeks the engine.
//
// The window owns the engine and rebuilds it on File->Open, so it can be opened
// with no bag (empty state) and load one interactively. The window geometry /
// replay options are fixed for the window's lifetime (they size the
// OccupancyBuffer); each opened bag builds a fresh engine with them.
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(
    BagLoadOptions load_opts, double window_m, double res, double max_range,
    double min_grazing_deg, QWidget * parent = nullptr);

  // Load a bag and (re)build the engine. On failure the message goes to the
  // status bar and the prior engine (if any) is left intact. Safe to call
  // repeatedly (File->Open).
  void openBag(const QString & bag_uri);

private slots:
  void onSeek(int index);
  void onParamEdited();
  void onOpen();

private:
  // One row of the data-driven param dock. Milestone B (after
  // unh_marine_perception#22-P1) swaps the knob set by editing the table that
  // builds these — not the widget code.
  struct Knob
  {
    std::string label;
    std::function<double()> get;                       // current value
    std::function<bool(double, std::string &)> apply;  // false + why on reject
    QDoubleSpinBox * box = nullptr;
  };

  void buildMenu();
  void buildParamDock();
  void syncToEngine();  // scrubber range + knob spinboxes + views for current engine
  void refreshViews();
  bool haveEngine() const {return engine_ != nullptr;}

  BagLoadOptions load_opts_;
  double window_m_;
  double res_;
  double max_range_;
  double min_grazing_deg_;

  std::unique_ptr<ReSimEngine> engine_;
  QLabel * seg_label_ = nullptr;
  QLabel * grid_label_ = nullptr;
  QSlider * scrubber_ = nullptr;
  int panel_px_ = 480;
  std::vector<Knob> knobs_;
};

}  // namespace marine_perception_tools

#endif  // MAIN_WINDOW_HPP_
