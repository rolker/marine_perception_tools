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

#include "main_window.hpp"

#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <QSlider>
#include <QStatusBar>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <initializer_list>
#include <string>
#include <utility>

#include "cv_qt.hpp"

namespace marine_perception_tools
{

MainWindow::MainWindow(ReSimEngine & engine, QWidget * parent)
: QMainWindow(parent), engine_(engine)
{
  setWindowTitle("sea_surface_tuner");

  seg_label_ = new QLabel(this);
  seg_label_->setAlignment(Qt::AlignCenter);
  seg_label_->setMinimumSize(320, 240);
  grid_label_ = new QLabel(this);
  grid_label_->setAlignment(Qt::AlignCenter);
  grid_label_->setMinimumSize(panel_px_, panel_px_);

  auto * images = new QHBoxLayout;
  images->addWidget(seg_label_, 1);
  images->addWidget(grid_label_, 1);

  scrubber_ = new QSlider(Qt::Horizontal, this);
  scrubber_->setRange(0, static_cast<int>(engine_.frameCount()) - 1);
  scrubber_->setValue(0);
  connect(scrubber_, &QSlider::valueChanged, this, &MainWindow::onSeek);

  auto * central = new QWidget(this);
  auto * layout = new QVBoxLayout(central);
  layout->addLayout(images, 1);
  layout->addWidget(scrubber_);
  setCentralWidget(central);

  buildParamDock();
  refreshViews();
}

void MainWindow::buildParamDock()
{
  // Data-driven knob table: a row per tunable field, addressed by member pointer.
  // Each setter copies the current params struct, mutates one field, and applies
  // it via the engine (which validates / bounds-checks and re-sims).
  using sea_surface_segmentation::AccumulateParams;
  using sea_surface_segmentation::OccupancyParams;

  // OccupancyParams knobs (validated by OccupancyBuffer::validate).
  for (const auto & [label, field] : std::initializer_list<
      std::pair<const char *, double OccupancyParams::*>>{
      {"decay_half_life_s", &OccupancyParams::decay_half_life_s},
      {"lethal_threshold", &OccupancyParams::lethal_threshold},
      {"obstacle_clamp", &OccupancyParams::obstacle_clamp},
      {"clear_floor", &OccupancyParams::clear_floor},
      {"free_threshold", &OccupancyParams::free_threshold}})
  {
    knobs_.push_back({label,
        [this, field] {return engine_.occupancyParams().*field;},
        [this, field](double v, std::string & why) {
          auto p = engine_.occupancyParams();
          p.*field = v;
          return engine_.setOccupancyParams(p, why);
        }, nullptr});
  }

  // AccumulateParams knobs (engine-bounds-checked; window geometry is not here).
  for (const auto & [label, field] : std::initializer_list<
      std::pair<const char *, double AccumulateParams::*>>{
      {"max_range", &AccumulateParams::max_range},
      {"min_grazing_angle_deg", &AccumulateParams::min_grazing_angle_deg},
      {"obstacle_prob_min", &AccumulateParams::obstacle_prob_min},
      {"max_evidence_step", &AccumulateParams::max_evidence_step}})
  {
    knobs_.push_back({label,
        [this, field] {return engine_.accumulateParams().*field;},
        [this, field](double v, std::string & why) {
          auto p = engine_.accumulateParams();
          p.*field = v;
          return engine_.setAccumulateParams(p, why);
        }, nullptr});
  }

  auto * panel = new QWidget;
  auto * form = new QFormLayout(panel);
  for (auto & knob : knobs_) {
    auto * box = new QDoubleSpinBox(panel);
    box->setRange(-1000.0, 1000.0);  // clear_floor/free_threshold can be negative
    box->setDecimals(3);
    box->setSingleStep(0.1);
    box->setValue(knob.get());
    connect(box, qOverload<double>(&QDoubleSpinBox::valueChanged),
      this, &MainWindow::onParamEdited);
    form->addRow(QString::fromStdString(knob.label), box);
    knob.box = box;
  }

  auto * dock = new QDockWidget("Parameters", this);
  dock->setWidget(panel);
  addDockWidget(Qt::RightDockWidgetArea, dock);
}

void MainWindow::onSeek(int index)
{
  engine_.seekTo(static_cast<std::size_t>(std::max(0, index)));
  refreshViews();
}

void MainWindow::onParamEdited()
{
  auto * box = qobject_cast<QDoubleSpinBox *>(sender());
  if (box == nullptr) {return;}
  for (auto & knob : knobs_) {
    if (knob.box != box) {continue;}
    std::string why;
    if (knob.apply(box->value(), why)) {
      refreshViews();
    } else {
      statusBar()->showMessage(QString::fromStdString(knob.label + ": " + why), 5000);
      box->blockSignals(true);   // revert without re-triggering this slot
      box->setValue(knob.get());
      box->blockSignals(false);
    }
    return;
  }
}

void MainWindow::refreshViews()
{
  const QImage seg = cvMatToQImage(engine_.currentMask(), /*bgr=*/false);
  seg_label_->setPixmap(
    QPixmap::fromImage(seg).scaledToHeight(panel_px_, Qt::SmoothTransformation));

  const QImage grid = cvMatToQImage(engine_.renderGrid(panel_px_), /*bgr=*/true);
  grid_label_->setPixmap(QPixmap::fromImage(grid));

  statusBar()->showMessage(
    QString("frame %1 / %2   t=%3 s")
    .arg(engine_.currentIndex())
    .arg(engine_.frameCount() - 1)
    .arg(engine_.currentStamp(), 0, 'f', 1));
}

}  // namespace marine_perception_tools
