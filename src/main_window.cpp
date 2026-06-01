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
#include <QtConcurrent>
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
#include <QPushButton>
#include <QResizeEvent>
#include <QSizePolicy>
#include <QSlider>
#include <QSplitter>
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
  double min_grazing_deg, double integration_halflives, double margin_s,
  double retention_s, QWidget * parent)
: QMainWindow(parent),
  load_opts_(std::move(load_opts)),
  window_m_(window_m),
  res_(res),
  max_range_(max_range),
  min_grazing_deg_(min_grazing_deg),
  integration_halflives_(integration_halflives),
  margin_s_(margin_s),
  retention_s_(retention_s)
{
  setWindowTitle("sea_surface_tuner");

  // Each row is a QWidget (so a vertical QSplitter can resize it) holding a
  // horizontal layout. Labels expand to fill (the splitter, not a fixed tile
  // size, governs their height) — minimums keep them usable when shrunk.
  auto make_row = [](std::array<QLabel *, kNumCameras> & labels,
    const std::function<QLabel *()> & mk) {
      auto * w = new QWidget;
      auto * h = new QHBoxLayout(w);
      h->setContentsMargins(0, 0, 0, 0);
      for (int i = 0; i < kNumCameras; ++i) {
        labels[i] = mk();
        h->addWidget(labels[i], 1);
      }
      return w;
    };
  auto expanding_label = [this] {
      auto * l = new QLabel(this);
      l->setAlignment(Qt::AlignCenter);
      l->setMinimumSize(120, 90);
      l->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      return l;
    };

  // Row 1: camera RGB.  Row 2: segmentation.  Both port|fwd|stbd|aft.
  auto * rgb_row = make_row(rgb_labels_, expanding_label);
  auto * seg_row = make_row(seg_labels_, expanding_label);

  // Row 3: recorded costmap (bag) | regenerated costmap (tuned).
  recorded_label_ = expanding_label();
  grid_label_ = expanding_label();
  auto * cm_widget = new QWidget;
  auto * cm_row = new QHBoxLayout(cm_widget);
  cm_row->setContentsMargins(0, 0, 0, 0);
  cm_row->addWidget(recorded_label_, 1);
  cm_row->addWidget(grid_label_, 1);

  // Vertical splitter so the operator chooses how much height each row gets.
  // Initial stretch favours the costmap row (the thing being tuned); all rows
  // stay non-collapsible so none can be dragged fully shut by accident.
  auto * rows = new QSplitter(Qt::Vertical, this);
  rows->addWidget(rgb_row);
  rows->addWidget(seg_row);
  rows->addWidget(cm_widget);
  rows->setChildrenCollapsible(false);
  rows->setStretchFactor(0, 2);
  rows->setStretchFactor(1, 2);
  rows->setStretchFactor(2, 3);
  // Dragging an internal handle resizes the rows but does NOT fire resizeEvent,
  // so re-fit the pixmaps to the new label sizes on splitterMoved too.
  connect(rows, &QSplitter::splitterMoved, this, [this] {
      if (haveEngine()) {refreshViews();}
    });

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
  layout->addWidget(rows, 1);       // splitter fills the window
  layout->addWidget(scrubber_);
  setCentralWidget(central);

  // Background window loads deliver a LoadResult back on the GUI thread.
  qRegisterMetaType<LoadResult>("LoadResult");
  connect(&load_watcher_, &QFutureWatcher<LoadResult>::finished,
    this, &MainWindow::onLoadFinished);

  buildMenu();
  buildParamDock();
  syncToEngine();  // empty state until a bag is opened
}

MainWindow::~MainWindow()
{
  // Block until an in-flight load finishes so its result can't be delivered to a
  // destroyed window. The worker holds the BagSession via a shared_ptr, so it
  // stays valid for the duration regardless of member teardown order.
  disconnect(&load_watcher_, nullptr, this, nullptr);
  if (load_watcher_.future().isRunning()) {
    load_watcher_.waitForFinished();
  }
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

  // Surface the compressed-segmentation caveat now (a scan-time property) — the
  // window load below is async, so we can't read it off the engine here.
  if (session_->usedCompressedSegmentation()) {
    statusBar()->showMessage(
      "Note: using compressed segmentation (raw topic absent) — if JPEG, the "
      "obstacle-probability channel is lossy; tuned values may not transfer.", 12000);
  }

  // Reset the dock to the empty baseline, then kick the initial window load
  // around t=0 in the background (cold start: the window clamps to the bag start
  // and warms forward — see the R5 caveat surfaced in refreshViews). The engine
  // installs via onLoadFinished; the GUI stays responsive meanwhile.
  syncToEngine();
  requestCoverage(0.0);
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

bool MainWindow::engineCovers(double t_s) const
{
  if (!engine_ || !session_) {return false;}
  const double lo = engine_->firstStamp() - session_->startTime();
  const double hi = engine_->lastStamp() - session_->startTime();
  return t_s >= lo - 1e-6 && t_s <= hi + 1e-6;
}

void MainWindow::requestCoverage(double t_s)
{
  if (!session_) {return;}

  // In-window fast path: the loaded engine already covers t_s, so seek
  // synchronously (cheap via the checkpoint store) — no reload, even if a
  // background load happens to be in flight for somewhere else.
  if (engineCovers(t_s)) {
    engine_->seekToStamp(session_->startTime() + t_s);
    refreshViews();
    return;
  }

  // Out of window. If a load is already running, just chase the latest target —
  // onLoadFinished re-launches for it. Otherwise start one now.
  if (load_in_flight_) {
    chase_target_s_ = t_s;
    return;
  }
  startLoad(t_s);
}

void MainWindow::startLoad(double t_s)
{
  load_in_flight_ = true;
  chase_target_s_ = -1.0;
  const BufferPlan plan = plan_buffer(t_s, bufferParams(), buffer_state_);

  // Grey the panes + status so a stale window isn't mistaken for the target
  // while the load runs (run feedback: clear out-of-date displays).
  for (int i = 0; i < kNumCameras; ++i) {
    rgb_labels_[i]->setPixmap(QPixmap());
    rgb_labels_[i]->setText(QString(kCameraLabels[i]) + " …");
    seg_labels_[i]->setPixmap(QPixmap());
    seg_labels_[i]->setText("…");
  }
  recorded_label_->setPixmap(QPixmap());
  recorded_label_->setText("loading…");
  grid_label_->setPixmap(QPixmap());
  grid_label_->setText("loading…");
  statusBar()->showMessage(
    QString("Loading %1 s window around t=%2 s …")
    .arg(plan.cache_hi - plan.cache_lo, 0, 'f', 0).arg(t_s, 0, 'f', 1));

  // Stage A: load the window + build a display-only engine (no preloaded bag).
  launchStage(t_s, ++request_id_, /*preloaded=*/nullptr);
}

void MainWindow::launchStage(
  double t_s, std::uint64_t id, std::shared_ptr<LoadedBag> preloaded)
{
  // Capture by value so nothing on the GUI thread is touched off-thread. (A
  // lambda, not a many-arg run() overload — Qt5's QtConcurrent::run can't deduce
  // that many bound args through a function pointer.)
  auto session = session_;
  const BufferParams params = bufferParams();
  const BufferState state = buffer_state_;
  const double window_m = window_m_, res = res_, max_range = max_range_;
  const double min_grazing = min_grazing_deg_;
  const sea_surface_segmentation::OccupancyParams occ =
    engine_ ? engine_->occupancyParams() : sea_surface_segmentation::OccupancyParams{};
  load_watcher_.setFuture(QtConcurrent::run(
      [session, params, state, t_s, window_m, res, max_range, min_grazing, occ, id,
      preloaded] {
        return loadWindowJob(
        session, params, state, t_s, window_m, res, max_range, min_grazing, occ, id,
        preloaded);
    }));
}

MainWindow::LoadResult MainWindow::loadWindowJob(
  std::shared_ptr<BagSession> session, BufferParams params, BufferState state,
  double t_s, double window_m, double res, double max_range, double min_grazing_deg,
  sea_surface_segmentation::OccupancyParams occ, std::uint64_t request_id,
  std::shared_ptr<LoadedBag> preloaded)
{
  // Worker thread: NO Qt / GUI access.
  LoadResult r;
  r.target_s = t_s;
  r.request_id = request_id;
  const BufferPlan plan = plan_buffer(t_s, params, state);
  r.cache_lo = plan.cache_lo;
  r.cache_hi = plan.cache_hi;
  try {
    if (!preloaded) {
      // Stage A: read the window (the slow I/O + H.265 decode) and build a
      // display-only engine — index set for the image views, NO warm-up replay.
      auto bag = std::make_shared<LoadedBag>(
        session->loadWindow(plan.cache_lo, plan.cache_hi));
      auto engine = std::make_shared<ReSimEngine>(
        *bag, window_m, res, max_range, occ, min_grazing_deg);
      engine->seekToStampDisplayOnly(session->startTime() + t_s);
      r.bag = std::move(bag);
      r.engine = std::move(engine);
      r.warmed = false;
    } else {
      // Stage B: build a fresh engine from the SAME bag (cv::Mat is refcounted —
      // copies frame headers, not pixels) and warm it to the target.
      auto engine = std::make_shared<ReSimEngine>(
        *preloaded, window_m, res, max_range, occ, min_grazing_deg);
      engine->seekToStamp(session->startTime() + t_s);
      r.bag = preloaded;
      r.engine = std::move(engine);
      r.warmed = true;
    }
  } catch (const std::exception & e) {
    r.error = e.what();
  }
  return r;
}

void MainWindow::onLoadFinished()
{
  load_in_flight_ = false;
  const LoadResult r = load_watcher_.result();

  // Superseded by a newer request: discard and serve the latest instead.
  if (r.request_id != request_id_) {
    if (chase_target_s_ >= 0.0) {
      const double t = chase_target_s_;
      chase_target_s_ = -1.0;
      requestCoverage(t);
    }
    return;
  }

  if (!r.engine) {
    statusBar()->showMessage(QString("Window load failed: ") +
      QString::fromStdString(r.error), 8000);
    return;
  }

  engine_ = r.engine;
  buffer_state_.has_cache = true;
  buffer_state_.cache_lo = r.cache_lo;
  buffer_state_.cache_hi = r.cache_hi;
  syncToEngine();  // paints all panes; regenerated shows "computing…" while stageA

  // A target requested mid-load supersedes — restart from stage A for it.
  if (chase_target_s_ >= 0.0) {
    const double t = chase_target_s_;
    chase_target_s_ = -1.0;
    requestCoverage(t);
    return;
  }

  // Stage A just delivered images + recorded costmap. Chain stage B to warm the
  // regenerated costmap from the same bag, without blocking the GUI.
  if (!r.warmed) {
    load_in_flight_ = true;
    statusBar()->showMessage("Computing regenerated costmap …");
    launchStage(r.target_s, r.request_id, r.bag);
  }
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
        [field](double v, OccupancyParams & occ, AccumulateParams &) {
          occ.*field = v;  // stage into the occupancy struct Apply will submit
        }, nullptr, false});
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
        [field](double v, OccupancyParams &, AccumulateParams & acc) {
          acc.*field = v;  // stage into the accumulate struct Apply will submit
        }, nullptr, false});
  }

  auto * panel = new QWidget;
  auto * form = new QFormLayout(panel);
  for (auto & knob : knobs_) {
    auto * box = new QDoubleSpinBox(panel);
    box->setRange(-1000.0, 1000.0);  // clear_floor/free_threshold can be negative
    box->setDecimals(3);
    box->setSingleStep(0.1);
    box->setValue(knob.read());
    // Editing marks the knob dirty (no re-sim — batched until Apply).
    connect(box, qOverload<double>(&QDoubleSpinBox::valueChanged),
      this, &MainWindow::onParamEdited);
    form->addRow(QString::fromStdString(knob.label), box);
    knob.box = box;
  }

  // Apply commits the dirty batch in one re-sim; Reset reverts to applied values.
  apply_btn_ = new QPushButton("Apply", panel);
  reset_btn_ = new QPushButton("Reset", panel);
  apply_btn_->setEnabled(false);
  reset_btn_->setEnabled(false);
  connect(apply_btn_, &QPushButton::clicked, this, &MainWindow::onApplyParams);
  connect(reset_btn_, &QPushButton::clicked, this, &MainWindow::onResetParams);
  auto * btn_row = new QHBoxLayout;
  btn_row->addWidget(apply_btn_);
  btn_row->addWidget(reset_btn_);
  form->addRow(btn_row);

  auto * dock = new QDockWidget("Parameters", this);
  dock->setWidget(panel);
  addDockWidget(Qt::RightDockWidgetArea, dock);
}

void MainWindow::setKnobDirty(Knob & knob, bool dirty)
{
  if (knob.box == nullptr) {return;}
  knob.dirty = dirty;
  // Distinct background for an edited-but-unapplied value (R2 dirty cue).
  knob.box->setStyleSheet(dirty ? "background: #fff3b0;" : QString());
}

void MainWindow::syncToEngine()
{
  const bool have = haveEngine();

  // The scrubber spans the whole bag in time (set in openBag), not the loaded
  // window's frames — so it is NOT reset here; a window reload must not move the
  // playhead. Only its enabled state tracks whether a bag is open.
  scrubber_->setEnabled(session_ != nullptr);

  // Reseed each spinbox from the (new) engine's applied params without
  // re-triggering edits, and clear any dirty cue — a fresh engine / window load
  // is a clean baseline.
  for (auto & knob : knobs_) {
    if (knob.box == nullptr) {continue;}
    knob.box->blockSignals(true);
    knob.box->setValue(knob.read());
    knob.box->blockSignals(false);
    knob.box->setEnabled(have);
    setKnobDirty(knob, false);
  }
  if (apply_btn_ != nullptr) {apply_btn_->setEnabled(false);}
  if (reset_btn_ != nullptr) {reset_btn_->setEnabled(false);}

  refreshViews();
}

void MainWindow::onSeek(int decisec)
{
  if (!session_) {return;}
  const double t_s = std::max(0, decisec) / 10.0;  // deciseconds -> bag-relative s

  // In-window: seek live (cheap via the checkpoint store) so dragging tracks the
  // costmap. Out-of-window: defer the (now background) load to release — a drag
  // sweeps through many out-of-window values, and starting a load for each would
  // thrash; one load for the value you settle on is right.
  if (engineCovers(t_s)) {
    engine_->seekToStamp(session_->startTime() + t_s);
    pending_seek_s_ = -1.0;
    refreshViews();
    return;
  }
  pending_seek_s_ = t_s;  // outside the window — load on release
  statusBar()->showMessage(
    QString("Release to load t=%1 s …").arg(t_s, 0, 'f', 1));
}

void MainWindow::onSeekReleased()
{
  if (!session_) {return;}
  // Commit the settled target. requestCoverage seeks in-window synchronously or
  // launches a background load (greying the panes) for an out-of-window target —
  // the GUI stays responsive either way, and a further scrub supersedes.
  const double t_s = (pending_seek_s_ >= 0.0) ?
    pending_seek_s_ : scrubber_->value() / 10.0;
  pending_seek_s_ = -1.0;
  requestCoverage(t_s);
}

void MainWindow::onParamEdited()
{
  // Editing does NOT re-simulate (too expensive per keystroke — D4 profiling).
  // Mark the knob dirty and arm Apply/Reset; the re-sim happens once on Apply.
  auto * box = qobject_cast<QDoubleSpinBox *>(sender());
  if (box == nullptr) {return;}
  for (auto & knob : knobs_) {
    if (knob.box != box) {continue;}
    setKnobDirty(knob, box->value() != knob.read());  // clean if edited back
    break;
  }
  bool any_dirty = false;
  for (const auto & k : knobs_) {
    any_dirty = any_dirty || k.dirty;
  }
  apply_btn_->setEnabled(any_dirty && haveEngine());
  reset_btn_->setEnabled(any_dirty);
}

void MainWindow::onApplyParams()
{
  if (!haveEngine()) {return;}
  // Build both staged structs from the applied baseline + every box's current
  // value, then submit as one batch — validate-both-before-apply, single re-sim.
  auto occ = engine_->occupancyParams();
  auto acc = engine_->accumulateParams();
  for (const auto & knob : knobs_) {
    if (knob.box == nullptr) {continue;}
    knob.stage(knob.box->value(), occ, acc);
  }
  std::string why;
  if (!engine_->setParams(occ, acc, why)) {
    // Rejected: leave engine + boxes untouched so the user can fix the offender.
    statusBar()->showMessage(QString("Apply rejected: ") + QString::fromStdString(why),
      6000);
    return;
  }
  // Applied: every box is now the baseline — clear dirty cues, disable buttons.
  for (auto & knob : knobs_) {
    setKnobDirty(knob, false);
  }
  apply_btn_->setEnabled(false);
  reset_btn_->setEnabled(false);
  refreshViews();
}

void MainWindow::onResetParams()
{
  // Revert each box to the applied value and clear its dirty cue. No re-sim — the
  // engine's params never changed (edits were staged in the boxes only).
  for (auto & knob : knobs_) {
    if (knob.box == nullptr) {continue;}
    knob.box->blockSignals(true);
    knob.box->setValue(knob.read());
    knob.box->blockSignals(false);
    setKnobDirty(knob, false);
  }
  apply_btn_->setEnabled(false);
  reset_btn_->setEnabled(false);
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

  // Scale a source image into a label at its current size, keeping aspect — so
  // the panes track the splitter (no fixed tile height). KeepAspectRatio leaves
  // letterbox margins, which is correct for the 4:3 camera tiles.
  auto fit = [](QLabel * lbl, const QImage & img) {
      lbl->setPixmap(
        QPixmap::fromImage(img).scaled(
          lbl->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    };

  // Rows 1–2: per-camera RGB + segmentation (latest at/before the current frame).
  for (int i = 0; i < kNumCameras; ++i) {
    const cv::Mat rgb = engine_->latestRgb(i);
    if (rgb.empty()) {
      rgb_labels_[i]->setPixmap(QPixmap());
      rgb_labels_[i]->setText(QString(kCameraLabels[i]) + " (no RGB)");
    } else {
      fit(rgb_labels_[i], cvMatToQImage(rgb, /*bgr=*/true));
    }
    const cv::Mat seg = engine_->latestMask(i);
    if (seg.empty()) {
      seg_labels_[i]->setPixmap(QPixmap());
      seg_labels_[i]->setText(QString(kCameraLabels[i]) + " (no seg)");
    } else {
      fit(seg_labels_[i], cvMatToQImage(seg, /*bgr=*/false));
    }
  }

  // Row 3: recorded costmap (bag) | regenerated (tuned) costmap, same window.
  // Render the boat-centred square at each pane's shorter side so it fills the
  // pane without distortion as the splitter resizes it. The recorded costmap is
  // available immediately (stage A); the regenerated one needs the warm-up replay
  // (stage B), so show "computing…" while the engine is display-only.
  const int rec_px = std::max(64, std::min(recorded_label_->width(), recorded_label_->height()));
  fit(recorded_label_, cvMatToQImage(engine_->renderRecorded(rec_px), /*bgr=*/true));
  if (engine_->displayOnly()) {
    grid_label_->setPixmap(QPixmap());
    grid_label_->setText("computing costmap…");
  } else {
    const int grid_px =
      std::max(64, std::min(grid_label_->width(), grid_label_->height()));
    fit(grid_label_, cvMatToQImage(engine_->renderGrid(grid_px), /*bgr=*/true));
  }

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

void MainWindow::resizeEvent(QResizeEvent * event)
{
  QMainWindow::resizeEvent(event);
  // Re-scale the panes to the new label sizes. Only when an engine is loaded —
  // refreshViews paints the empty-state text otherwise, which needs no resize.
  if (haveEngine()) {refreshViews();}
}

}  // namespace marine_perception_tools
