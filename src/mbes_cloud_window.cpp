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

  connect(&watcher_, &QFutureWatcher<CloudLoadOutcome>::finished,
    this, &MbesCloudWindow::onLoaded);
  const auto snapshot = passes_;   // worker owns its own copy
  watcher_.setFuture(
    QtConcurrent::run([snapshot]() {return load_cloud_passes(snapshot);}));
}

MbesCloudWindow::~MbesCloudWindow()
{
  watcher_.waitForFinished();
}

void MbesCloudWindow::onLoaded()
{
  const CloudLoadOutcome out = watcher_.result();

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
