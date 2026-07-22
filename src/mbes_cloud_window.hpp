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

#ifndef MBES_CLOUD_WINDOW_HPP_
#define MBES_CLOUD_WINDOW_HPP_

#include <QFutureWatcher>
#include <QMainWindow>
#include <QStringList>
#include <QTreeWidget>

#include <cstdint>
#include <string>
#include <vector>

#include "mbes_geometry.hpp"
#include "mbes_pass_loader.hpp"
#include "point_cloud_view.hpp"

namespace marine_perception_tools
{

// The multi-pass MBES drill-down (#21, explorer stage 3): the selected passes'
// soundings in one 3D PointCloudView, coloured per pass (golden-angle hues), a
// legend pairing each colour with its pass, and a status line carrying the
// load outcome. Loading runs on a worker thread (windowed bag reads via
// read_mbes_window); cross-bag passes are reprojected through the earth
// anchor into the FIRST pass's world frame. Self-contained — the overview
// stays open as the navigation hub.
class MbesCloudWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MbesCloudWindow(
    std::vector<CloudPassInfo> passes, QWidget * parent = nullptr);
  // The worker is self-contained (static fn + snapshot copy), but don't let
  // its QtConcurrent thread outlive the window/QApplication at shutdown.
  ~MbesCloudWindow() override;

private:
  void onLoaded();

  std::vector<CloudPassInfo> passes_;
  PointCloudView * view_ = nullptr;
  QTreeWidget * legend_ = nullptr;
  QFutureWatcher<CloudLoadOutcome> watcher_;
};

}  // namespace marine_perception_tools

#endif  // MBES_CLOUD_WINDOW_HPP_
