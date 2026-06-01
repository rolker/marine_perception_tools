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
#include <QApplication>
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

  auto make_label = [this](int w, int h) {
      auto * l = new QLabel(this);
      l->setAlignment(Qt::AlignCenter);
      l->setMinimumSize(w, h);
      return l;
    };
  const int tile_w = cam_tile_h_ * 4 / 3;  // 4:3 camera/seg tiles

  // Row 1: camera RGB.  Row 2: segmentation.  Both port|fwd|stbd|aft.
  auto * rgb_row = new QHBoxLayout;
  auto * seg_row = new QHBoxLayout;
  for (int i = 0; i < kNumCameras; ++i) {
    rgb_labels_[i] = make_label(tile_w, cam_tile_h_);
    seg_labels_[i] = make_label(tile_w, cam_tile_h_);
    rgb_row->addWidget(rgb_labels_[i], 1);
    seg_row->addWidget(seg_labels_[i], 1);
  }

  // Row 3: recorded costmap (bag) | regenerated costmap (tuned).
  recorded_label_ = make_label(panel_px_, panel_px_);
  grid_label_ = make_label(panel_px_, panel_px_);
  auto * cm_row = new QHBoxLayout;
  cm_row->addWidget(recorded_label_, 1);
  cm_row->addWidget(grid_label_, 1);

  // Whole-bag time scrubber in deciseconds (0.1 s steps); range set on open.
  // valueChanged fires continuously during a drag — handled live only when the
  // target is inside the loaded window (cheap seek). An out-of-window target is
  // deferred to sliderReleased so a multi-second window reload never fires mid-
  // drag (R8). Also commit on release for keyboard/wheel steps that don't drag.
  scrubber_ = new QSlider(Qt::Horizontal, this);
  scrubber_->setRange(0, 0);
  scrubber_->setValue(0);
  connect(scrubber_, &QSlider::valueChanged, this, &MainWindow::onSeek);
  connect(scrubber_, &QSlider::sliderReleased, this, &MainWindow::onSeekReleased);

  auto * central = new QWidget(this);
  auto * layout = new QVBoxLayout(central);
  layout->addLayout(rgb_row);
  layout->addLayout(seg_row);
  layout->addLayout(cm_row, 1);
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
  statusBar()->showMessage("Opening " + bag_uri + " …");
  // The one full scan (TF cache + models + bounds) happens here; windowed reads
  // are cheap thereafter. Build into a local first so a bad pick doesn't blank a
  // good session.
  std::unique_ptr<BagSession> session;
  try {
    session = std::make_unique<BagSession>(bag_uri.toStdString(), load_opts_);
  } catch (const std::exception & e) {
    statusBar()->showMessage(QString("Open failed: ") + e.what(), 8000);
    return;
  }
  session_ = std::move(session);
  buffer_state_ = BufferState{};  // no cache yet — the first ensureCovers loads
  engine_.reset();

  // Whole-bag time scrubber, in deciseconds (0.1 s steps) over [0, duration].
  scrubber_->blockSignals(true);
  scrubber_->setRange(0, static_cast<int>(session_->duration_s() * 10.0));
  scrubber_->setValue(0);
  scrubber_->blockSignals(false);

  // Load the initial window around t=0 (cold start: the window clamps to the bag
  // start and warms forward — see the R5 caveat surfaced in refreshViews).
  if (!ensureCovers(0.0)) {return;}  // ensureCovers reports its own failure
  syncToEngine();
  if (engine_ && engine_->usedCompressedSegmentation()) {
    statusBar()->showMessage(
      "Note: using compressed segmentation (raw topic absent) — if JPEG, the "
      "obstacle-probability channel is lossy; tuned values may not transfer.", 12000);
  }
}

double MainWindow::integrationSeconds() const
{
  // Warm-up tracks the decay half-life so changing the knob grows/shrinks the
  // window. Fall back to the OccupancyParams default when no engine exists yet.
  const double half_life = engine_ ? engine_->occupancyParams().decay_half_life_s :
    sea_surface_segmentation::OccupancyParams{}.decay_half_life_s;
  return integration_halflives_ * half_life;
}

BufferParams MainWindow::bufferParams() const
{
  BufferParams p;
  p.integration_s = integrationSeconds();
  p.margin_s = margin_s_;
  p.retention_s = retention_s_;
  p.bag_lo = 0.0;
  p.bag_hi = session_ ? session_->duration_s() : 0.0;
  return p;
}

bool MainWindow::ensureCovers(double t_s)
{
  if (!session_) {return false;}

  // In-window fast path: the target is already covered by the loaded engine, so
  // just seek (cheap via the checkpoint store) — no policy, no reload.
  if (engine_ && t_s >= engine_->firstStamp() - session_->startTime() - 1e-6 &&
    t_s <= engine_->lastStamp() - session_->startTime() + 1e-6)
  {
    engine_->seekToStamp(session_->startTime() + t_s);
    return true;
  }

  const BufferPlan plan = plan_buffer(t_s, bufferParams(), buffer_state_);
  if (!plan.read) {
    // Policy says the replay span is already cached but the engine doesn't cover
    // it (e.g. just constructed) — shouldn't happen in practice; fall through to
    // a load of the planned cache span to be safe.
  }

  statusBar()->showMessage(
    QString("Warming up %1 s window around t=%2 s …")
    .arg(plan.cache_hi - plan.cache_lo, 0, 'f', 0).arg(t_s, 0, 'f', 1));
  QApplication::processEvents();  // paint the status before the blocking load

  std::unique_ptr<ReSimEngine> engine;
  try {
    LoadedBag bag = session_->loadWindow(plan.cache_lo, plan.cache_hi);
    engine = std::make_unique<ReSimEngine>(
      std::move(bag), window_m_, res_, max_range_,
      engine_ ? engine_->occupancyParams() : sea_surface_segmentation::OccupancyParams{},
      min_grazing_deg_);
  } catch (const std::exception & e) {
    statusBar()->showMessage(QString("Window load failed: ") + e.what(), 8000);
    return false;
  }
  engine_ = std::move(engine);
  buffer_state_.has_cache = true;
  buffer_state_.cache_lo = plan.cache_lo;
  buffer_state_.cache_hi = plan.cache_hi;
  engine_->seekToStamp(session_->startTime() + t_s);
  return true;
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

  // The scrubber spans the whole bag in time (set in openBag), not the loaded
  // window's frames — so it is NOT reset here; a window reload must not move the
  // playhead. Only its enabled state tracks whether a bag is open.
  scrubber_->setEnabled(session_ != nullptr);

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

void MainWindow::onSeek(int decisec)
{
  if (!session_) {return;}
  const double t_s = std::max(0, decisec) / 10.0;  // deciseconds -> bag-relative s

  // In-window: seek live (cheap via the checkpoint store) so dragging tracks the
  // costmap. Out-of-window: defer the reload to release (R8) — stash the target
  // and surface that a release will load it, but don't block mid-drag.
  if (engine_) {
    const double rel_lo = engine_->firstStamp() - session_->startTime();
    const double rel_hi = engine_->lastStamp() - session_->startTime();
    if (t_s >= rel_lo - 1e-6 && t_s <= rel_hi + 1e-6) {
      engine_->seekToStamp(session_->startTime() + t_s);
      pending_seek_s_ = -1.0;
      refreshViews();
      return;
    }
  }
  pending_seek_s_ = t_s;  // outside the window — commit on release
  statusBar()->showMessage(
    QString("Release to load t=%1 s …").arg(t_s, 0, 'f', 1));
}

void MainWindow::onSeekReleased()
{
  if (!session_) {return;}
  // Commit a deferred out-of-window target; if none pending (the drag stayed in
  // window, or a keyboard/wheel step landed in window), there's nothing to do.
  const double t_s = (pending_seek_s_ >= 0.0) ?
    pending_seek_s_ : scrubber_->value() / 10.0;
  pending_seek_s_ = -1.0;
  if (engine_) {
    const double rel_lo = engine_->firstStamp() - session_->startTime();
    const double rel_hi = engine_->lastStamp() - session_->startTime();
    if (t_s >= rel_lo - 1e-6 && t_s <= rel_hi + 1e-6) {
      engine_->seekToStamp(session_->startTime() + t_s);
      refreshViews();
      return;
    }
  }
  if (ensureCovers(t_s)) {refreshViews();}
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
    for (int i = 0; i < kNumCameras; ++i) {
      rgb_labels_[i]->setPixmap(QPixmap());
      rgb_labels_[i]->setText(kCameraLabels[i]);
      seg_labels_[i]->setPixmap(QPixmap());
      seg_labels_[i]->setText("—");
    }
    recorded_label_->setPixmap(QPixmap());
    recorded_label_->setText("recorded costmap");
    grid_label_->setPixmap(QPixmap());
    grid_label_->setText("File → Open Bag…");
    statusBar()->showMessage("No bag loaded — File → Open Bag…");
    return;
  }

  // Rows 1–2: per-camera RGB + segmentation (latest at/before the current frame).
  for (int i = 0; i < kNumCameras; ++i) {
    const cv::Mat rgb = engine_->latestRgb(i);
    if (rgb.empty()) {
      rgb_labels_[i]->setPixmap(QPixmap());
      rgb_labels_[i]->setText(QString(kCameraLabels[i]) + " (no RGB)");
    } else {
      rgb_labels_[i]->setPixmap(
        QPixmap::fromImage(cvMatToQImage(rgb, /*bgr=*/true))
        .scaledToHeight(cam_tile_h_, Qt::SmoothTransformation));
    }
    const cv::Mat seg = engine_->latestMask(i);
    if (seg.empty()) {
      seg_labels_[i]->setPixmap(QPixmap());
      seg_labels_[i]->setText(QString(kCameraLabels[i]) + " (no seg)");
    } else {
      seg_labels_[i]->setPixmap(
        QPixmap::fromImage(cvMatToQImage(seg, /*bgr=*/false))
        .scaledToHeight(cam_tile_h_, Qt::SmoothTransformation));
    }
  }

  // Row 3: recorded costmap (bag) | regenerated (tuned) costmap, same window.
  const QImage recorded = cvMatToQImage(engine_->renderRecorded(panel_px_), /*bgr=*/true);
  recorded_label_->setPixmap(QPixmap::fromImage(recorded));
  const QImage grid = cvMatToQImage(engine_->renderGrid(panel_px_), /*bgr=*/true);
  grid_label_->setPixmap(QPixmap::fromImage(grid));

  // Bag-relative time (raw header stamps are epoch seconds — not meaningful to a
  // user; Copilot 1.4 / R7). The warm-up actually behind the current frame is
  // currentStamp - firstStamp; near the bag start it's shorter than the full
  // integration, so the regenerated costmap has less history than the recorded
  // one — surface that so an edge artifact isn't read as a parameter effect (R5).
  const double t_rel = session_ ? engine_->currentStamp() - session_->startTime() :
    engine_->currentStamp();
  const double warmup = engine_->currentStamp() - engine_->firstStamp();
  statusBar()->showMessage(
    QString("t=%1 s   windowed: warm-up %2 s (regenerated costmap is not full history)")
    .arg(t_rel, 0, 'f', 1)
    .arg(warmup, 0, 'f', 0));
}

}  // namespace marine_perception_tools
