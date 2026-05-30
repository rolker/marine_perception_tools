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

#include <QAction>
#include <QDir>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QKeySequence>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QPixmap>
#include <QSlider>
#include <QStatusBar>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <exception>
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>

#include "cv_qt.hpp"

namespace marine_perception_tools
{

MainWindow::MainWindow(
  BagLoadOptions load_opts, double window_m, double res, double max_range,
  double min_grazing_deg, QWidget * parent)
: QMainWindow(parent),
  load_opts_(std::move(load_opts)),
  window_m_(window_m),
  res_(res),
  max_range_(max_range),
  min_grazing_deg_(min_grazing_deg)
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
  scrubber_->setRange(0, 0);
  scrubber_->setValue(0);
  connect(scrubber_, &QSlider::valueChanged, this, &MainWindow::onSeek);

  auto * central = new QWidget(this);
  auto * layout = new QVBoxLayout(central);
  layout->addLayout(images, 1);
  layout->addWidget(scrubber_);
  setCentralWidget(central);

  buildMenu();
  buildParamDock();
  syncToEngine();  // empty state until a bag is opened
}

void MainWindow::buildMenu()
{
  QMenu * file = menuBar()->addMenu("&File");

  QAction * open = file->addAction("&Open Bag…");
  open->setShortcut(QKeySequence::Open);
  connect(open, &QAction::triggered, this, &MainWindow::onOpen);

  file->addSeparator();
  QAction * quit = file->addAction("&Quit");
  quit->setShortcut(QKeySequence::Quit);
  connect(quit, &QAction::triggered, this, &MainWindow::close);
}

void MainWindow::onOpen()
{
  // A rosbag2 is a directory containing metadata.yaml; pick the directory.
  const QString dir = QFileDialog::getExistingDirectory(
    this, "Open rosbag2 directory", QDir::homePath());
  if (dir.isEmpty()) {return;}  // cancelled
  openBag(dir);
}

void MainWindow::openBag(const QString & bag_uri)
{
  statusBar()->showMessage("Loading " + bag_uri + " …");
  std::unique_ptr<ReSimEngine> engine;
  try {
    LoadedBag bag = load_forward_camera(bag_uri.toStdString(), load_opts_);
    engine = std::make_unique<ReSimEngine>(
      std::move(bag), window_m_, res_, max_range_,
      sea_surface_segmentation::OccupancyParams{}, min_grazing_deg_);
  } catch (const std::exception & e) {
    // Keep the prior engine (if any) so a bad pick doesn't blank a good session.
    statusBar()->showMessage(QString("Open failed: ") + e.what(), 8000);
    return;
  }
  engine_ = std::move(engine);
  syncToEngine();
}

void MainWindow::buildParamDock()
{
  // Data-driven knob table: a row per tunable field, addressed by member pointer.
  // Each setter copies the current params struct, mutates one field, and applies
  // it via the engine (which validates / bounds-checks and re-sims). Getters fall
  // back to the struct default when no bag is loaded, so the dock shows sane
  // values in the empty state.
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
        [this, field] {
          return engine_ ? engine_->occupancyParams().*field : OccupancyParams{}.*field;
        },
        [this, field](double v, std::string & why) {
          if (!engine_) {why = "no bag loaded"; return false;}
          auto p = engine_->occupancyParams();
          p.*field = v;
          return engine_->setOccupancyParams(p, why);
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
        [this, field] {
          return engine_ ? engine_->accumulateParams().*field : AccumulateParams{}.*field;
        },
        [this, field](double v, std::string & why) {
          if (!engine_) {why = "no bag loaded"; return false;}
          auto p = engine_->accumulateParams();
          p.*field = v;
          return engine_->setAccumulateParams(p, why);
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

void MainWindow::syncToEngine()
{
  const bool have = haveEngine();

  // Scrubber range + reset to frame 0 without firing onSeek mid-rebuild.
  scrubber_->blockSignals(true);
  scrubber_->setRange(0, have ? static_cast<int>(engine_->frameCount()) - 1 : 0);
  scrubber_->setValue(0);
  scrubber_->blockSignals(false);
  scrubber_->setEnabled(have);

  // Reseed each spinbox from the (new) engine's params without re-triggering edits.
  for (auto & knob : knobs_) {
    if (knob.box == nullptr) {continue;}
    knob.box->blockSignals(true);
    knob.box->setValue(knob.get());
    knob.box->blockSignals(false);
    knob.box->setEnabled(have);
  }

  refreshViews();
}

void MainWindow::onSeek(int index)
{
  if (!haveEngine()) {return;}
  engine_->seekTo(static_cast<std::size_t>(std::max(0, index)));
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
  if (!haveEngine()) {
    seg_label_->setPixmap(QPixmap());
    grid_label_->setPixmap(QPixmap());
    seg_label_->setText("No bag loaded");
    grid_label_->setText("File → Open Bag…");
    statusBar()->showMessage("No bag loaded — File → Open Bag…");
    return;
  }

  const QImage seg = cvMatToQImage(engine_->currentMask(), /*bgr=*/false);
  seg_label_->setPixmap(
    QPixmap::fromImage(seg).scaledToHeight(panel_px_, Qt::SmoothTransformation));

  const QImage grid = cvMatToQImage(engine_->renderGrid(panel_px_), /*bgr=*/true);
  grid_label_->setPixmap(QPixmap::fromImage(grid));

  statusBar()->showMessage(
    QString("frame %1 / %2   t=%3 s")
    .arg(engine_->currentIndex())
    .arg(engine_->frameCount() - 1)
    .arg(engine_->currentStamp(), 0, 'f', 1));
}

}  // namespace marine_perception_tools
