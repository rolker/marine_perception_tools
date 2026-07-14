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

#ifndef SURVEY_OVERVIEW_WINDOW_HPP_
#define SURVEY_OVERVIEW_WINDOW_HPP_

#include <QLabel>
#include <QMainWindow>
#include <QTreeWidget>

#include <memory>
#include <string>

#include "survey_index_bridge.hpp"
#include "survey_overview_canvas.hpp"

namespace marine_perception_tools
{

// The explorer's front door (#258 stage 2): the survey as a store-tile map,
// click where the question is, get the passes that saw it, open one cued
// (jump-to-pass, #17). Left: SurveyOverviewCanvas with the colormapped
// bathymetry tiles. Right: pass list grouped by bag; double-click opens a
// SidescanViewerWindow cued to that pass's time window.
class SurveyOverviewWindow : public QMainWindow
{
  Q_OBJECT

public:
  // index_path: survey_index.db. stores_dir: a directory containing GGGS
  // store GeoTIFF tiles (e.g. <stores>/bathymetry/survey). Throws
  // std::runtime_error when the index can't be opened; a missing/empty
  // stores dir degrades to an empty map (the pass query still works).
  SurveyOverviewWindow(
    const std::string & index_path, const std::string & stores_dir,
    QWidget * parent = nullptr);

private Q_SLOTS:
  void onMapClicked(double lat, double lon);
  void onPassActivated(QTreeWidgetItem * item, int column);
  void onHoverGeo(double lat, double lon);

private:
  void loadStoreTiles(const std::string & stores_dir);

  std::unique_ptr<SurveyIndexBridge> bridge_;
  SurveyOverviewCanvas * canvas_ = nullptr;
  QTreeWidget * pass_list_ = nullptr;
  QLabel * status_ = nullptr;
  QLabel * hover_ = nullptr;
};

}  // namespace marine_perception_tools

#endif  // SURVEY_OVERVIEW_WINDOW_HPP_
