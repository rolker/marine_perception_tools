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

#include "mbes_cloud_window.hpp"

#include <QColor>
#include <QHeaderView>
#include <QPixmap>
#include <QSplitter>
#include <QStatusBar>
#include <QString>
#include <QtConcurrent>

#include <utility>
#include <vector>

#include "mbes_window_reader.hpp"

namespace marine_perception_tools
{

MbesCloudWindow::MbesCloudWindow(
  std::vector<CloudPassInfo> passes, QWidget * parent)
: QMainWindow(parent), passes_(std::move(passes))
{
  setWindowTitle(QString("MBES cloud — %1 pass%2")
    .arg(passes_.size()).arg(passes_.size() == 1 ? "" : "es"));

  auto * splitter = new QSplitter(this);
  view_ = new PointCloudView(splitter);
  view_->setColorMode(PointCloudView::ColorMode::Pass);
  legend_ = new QTreeWidget(splitter);
  legend_->setHeaderLabels({"Pass", "Soundings"});
  legend_->setRootIsDecorated(false);
  legend_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  splitter->addWidget(view_);
  splitter->addWidget(legend_);
  splitter->setStretchFactor(0, 1);
  splitter->setSizes({900, 340});
  setCentralWidget(splitter);
  resize(1240, 800);

  statusBar()->showMessage(
    QString("Loading %1 pass%2…").arg(passes_.size())
    .arg(passes_.size() == 1 ? "" : "es"));

  connect(&watcher_, &QFutureWatcher<LoadOutcome>::finished,
    this, &MbesCloudWindow::onLoaded);
  const auto snapshot = passes_;   // worker owns its own copy
  watcher_.setFuture(QtConcurrent::run([snapshot]() {return loadPasses(snapshot);}));
}

MbesCloudWindow::LoadOutcome MbesCloudWindow::loadPasses(
  const std::vector<CloudPassInfo> & passes)
{
  LoadOutcome out;
  out.pass_clouds.resize(passes.size());
  out.sounding_counts.assign(passes.size(), 0);

  // The FIRST pass's bag defines the reference world frame; other bags'
  // soundings reproject through the earth anchor (plan #21 design decision —
  // local map origins are not guaranteed identical across recordings).
  bool have_ref = false;
  std::string ref_bag;
  std::string ref_frame;
  bool ref_has_geo = false;
  geometry_msgs::msg::TransformStamped ref_earth_from_world;

  for (std::size_t i = 0; i < passes.size(); ++i) {
    const auto & pass = passes[i];
    MbesWindowResult res;
    try {
      res = read_mbes_window(pass.bag_path, pass.t_start_ns, pass.t_end_ns);
    } catch (const std::exception & e) {
      ++out.skipped_passes;
      out.notes << QString("%1: bag read failed (%2)")
        .arg(QString::fromStdString(pass.label)).arg(e.what());
      continue;
    }
    out.skipped_pings += res.skipped_pings;
    if (res.world_soundings.empty()) {
      out.notes << QString("%1: no soundings in window")
        .arg(QString::fromStdString(pass.label));
      continue;
    }

    if (!have_ref) {
      have_ref = true;
      ref_bag = pass.bag_path;
      ref_frame = res.world_frame;
      ref_has_geo = res.has_geo;
      ref_earth_from_world = res.earth_from_world;
    }

    FrameReprojection reproject;   // identity by default
    if (pass.bag_path != ref_bag) {
      if (ref_has_geo && res.has_geo) {
        reproject = make_reprojection(ref_earth_from_world, res.earth_from_world);
      } else if (res.world_frame != ref_frame) {
        // No geo anchor and a different frame: placement would be a guess —
        // skip visibly rather than mis-place soundings.
        ++out.skipped_passes;
        out.notes << QString("%1: no geo anchor to relate frame '%2' to '%3' — skipped")
          .arg(QString::fromStdString(pass.label))
          .arg(QString::fromStdString(res.world_frame))
          .arg(QString::fromStdString(ref_frame));
        continue;
      }
      // Same frame NAME without geo anchors: assume the shared local frame
      // (single-deployment recordings); identity.
    }

    auto & cloud = out.pass_clouds[i];
    cloud = std::move(res.world_soundings);
    for (auto & s : cloud) {
      apply_reprojection(reproject, s.x, s.y, s.z);
    }
    out.sounding_counts[i] = static_cast<int>(cloud.size());
  }
  return out;
}

void MbesCloudWindow::onLoaded()
{
  const LoadOutcome out = watcher_.result();

  view_->resetView();
  view_->setMultiPassPoints(out.pass_clouds);

  legend_->clear();
  int total = 0;
  for (std::size_t i = 0; i < passes_.size(); ++i) {
    auto * item = new QTreeWidgetItem(legend_, {
        QString::fromStdString(passes_[i].label),
        QString::number(out.sounding_counts[i])});
    QPixmap swatch(12, 12);
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
    pass_color(static_cast<int>(i), r, g, b);
    swatch.fill(QColor::fromRgbF(r, g, b));
    item->setIcon(0, swatch);
    if (out.sounding_counts[i] == 0) {
      item->setDisabled(true);
    }
    total += out.sounding_counts[i];
  }

  QString message = QString("%1 soundings from %2 pass%3")
    .arg(total).arg(passes_.size()).arg(passes_.size() == 1 ? "" : "es");
  if (out.skipped_passes > 0) {
    message += QString(", %1 pass%2 skipped")
      .arg(out.skipped_passes).arg(out.skipped_passes == 1 ? "" : "es");
  }
  if (out.skipped_pings > 0) {
    message += QString(", %1 pings without TF").arg(out.skipped_pings);
  }
  if (!out.notes.isEmpty()) {
    message += " — " + out.notes.join("; ");
  }
  statusBar()->showMessage(message);
}

}  // namespace marine_perception_tools
