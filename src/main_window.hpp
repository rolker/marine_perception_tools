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

#include <functional>
#include <string>
#include <vector>

#include "resim_engine.hpp"

class QDoubleSpinBox;
class QLabel;
class QSlider;

namespace marine_perception_tools
{

// The sea_surface_tuner main window: a forward-camera segmentation pane beside
// the re-simulated costmap, a frame scrubber, and a parameter dock. Editing a
// knob re-simulates and refreshes the costmap; scrubbing seeks the engine.
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(ReSimEngine & engine, QWidget * parent = nullptr);

private slots:
  void onSeek(int index);
  void onParamEdited();

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

  void buildParamDock();
  void refreshViews();

  ReSimEngine & engine_;
  QLabel * seg_label_ = nullptr;
  QLabel * grid_label_ = nullptr;
  QSlider * scrubber_ = nullptr;
  int panel_px_ = 480;
  std::vector<Knob> knobs_;
};

}  // namespace marine_perception_tools

#endif  // MAIN_WINDOW_HPP_
