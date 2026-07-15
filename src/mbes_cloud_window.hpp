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
#include "point_cloud_view.hpp"

namespace marine_perception_tools
{

// One pass to load into the cloud: the bag it lives in, its time window (from
// the survey index), and a human label for the legend.
struct CloudPassInfo
{
  std::string bag_path;
  std::string label;
  std::int64_t t_start_ns = 0;
  std::int64_t t_end_ns = 0;
};

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
  // Everything the worker thread hands back to the UI thread in one piece.
  struct LoadOutcome
  {
    std::vector<std::vector<MbesSounding>> pass_clouds;  // reference world frame
    std::vector<int> sounding_counts;   // per input pass (0 = empty/skipped)
    QStringList notes;                  // per-pass problems, human-readable
    int skipped_passes = 0;             // passes dropped (frame not resolvable)
    int skipped_pings = 0;              // pings dropped (no TF), summed
  };

  static LoadOutcome loadPasses(const std::vector<CloudPassInfo> & passes);
  void onLoaded();

  std::vector<CloudPassInfo> passes_;
  PointCloudView * view_ = nullptr;
  QTreeWidget * legend_ = nullptr;
  QFutureWatcher<LoadOutcome> watcher_;
};

}  // namespace marine_perception_tools

#endif  // MBES_CLOUD_WINDOW_HPP_
