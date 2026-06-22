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

#include "sidescan_viewer_window.hpp"

#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPointF>
#include <QSlider>
#include <QString>
#include <QtConcurrent>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>
#include <vector>

#include "coverage_raster.hpp"
#include "distance_buffer_policy.hpp"
#include "sidescan_canvas.hpp"
#include "sidescan_geometry.hpp"

namespace marine_perception_tools
{
namespace
{

// Maximum slant range (≈ far ground range) of a ping's last sample, used to pad
// the swath bounding box.
double ping_max_range(const SidescanPing & p)
{
  if (p.amplitudes.empty() || p.geometry.metres_per_sample <= 0.0) {return 0.0;}
  return slant_range_at(p.amplitudes.size(), p.geometry.sample0, p.geometry.metres_per_sample);
}

QImage render_coverage(const CoverageRaster & r)
{
  QImage img(r.width(), r.height(), QImage::Format_ARGB32);
  img.fill(Qt::transparent);
  for (int row = 0; row < r.height(); ++row) {
    // Raster row 0 = south (min north); image row 0 = north edge (top).
    const int img_row = r.height() - 1 - row;
    for (int col = 0; col < r.width(); ++col) {
      const float a = r.amplitudeAt(col, row);
      if (a < 0.0f) {continue;}  // uncovered -> transparent
      const int g = std::clamp(static_cast<int>(a * 255.0f + 0.5f), 0, 255);
      img.setPixel(col, img_row, qRgba(g, g, g, 255));
    }
  }
  return img;
}

}  // namespace

SidescanViewerWindow::SidescanViewerWindow(QWidget * parent)
: QMainWindow(parent)
{
  setWindowTitle("Sidescan Target Viewer");

  canvas_ = new SidescanCanvas(this);

  scrub_ = new QSlider(Qt::Horizontal, this);
  scrub_->setRange(0, 1000);
  scrub_->setEnabled(false);

  grid_spin_ = new QDoubleSpinBox(this);
  grid_spin_->setRange(1.0, 1000.0);
  grid_spin_->setValue(10.0);
  grid_spin_->setSuffix(" m");

  window_spin_ = new QDoubleSpinBox(this);
  window_spin_->setRange(10.0, 1000.0);
  window_spin_->setValue(window_len_m_);
  window_spin_->setSuffix(" m");

  status_ = new QLabel("Open a bag to begin (File → Open Bag).", this);

  auto * controls = new QWidget(this);
  auto * crow = new QHBoxLayout(controls);
  crow->addWidget(new QLabel("Distance:", this));
  crow->addWidget(scrub_, 1);
  crow->addWidget(new QLabel("Grid:", this));
  crow->addWidget(grid_spin_);
  crow->addWidget(new QLabel("Window:", this));
  crow->addWidget(window_spin_);

  auto * central = new QWidget(this);
  auto * col = new QVBoxLayout(central);
  col->addWidget(canvas_, 1);
  col->addWidget(controls);
  col->addWidget(status_);
  setCentralWidget(central);

  auto * file_menu = menuBar()->addMenu("&File");
  file_menu->addAction("&Open Bag…", this, &SidescanViewerWindow::onOpenBag);
  file_menu->addAction("&Fit View", this, [this]() {canvas_->resetView();});
  file_menu->addSeparator();
  file_menu->addAction("E&xit", this, &QWidget::close);

  connect(&load_watcher_, &QFutureWatcher<SidescanLoadResult>::finished,
    this, &SidescanViewerWindow::onLoadFinished);
  connect(scrub_, &QSlider::valueChanged, this, &SidescanViewerWindow::onScrubChanged);
  connect(grid_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, &SidescanViewerWindow::onGridSpacingChanged);
  connect(window_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, &SidescanViewerWindow::onWindowLengthChanged);

  canvas_->setGridSpacing(grid_spin_->value());
  resize(1100, 760);
}

void SidescanViewerWindow::onOpenBag()
{
  const QString dir = QFileDialog::getExistingDirectory(this, "Open ROS 2 bag directory");
  if (!dir.isEmpty()) {openBag(dir.toStdString());}
}

void SidescanViewerWindow::openBag(const std::string & bag_uri)
{
  if (loading_) {return;}  // a load is already in flight
  loading_ = true;
  scrub_->setEnabled(false);
  status_->setText(QString("Loading %1 …").arg(QString::fromStdString(bag_uri)));

  // Read the bag off the UI thread so the window stays responsive (no WM
  // "application not responding"). Errors are returned, not thrown across threads.
  load_watcher_.setFuture(QtConcurrent::run([bag_uri]() -> SidescanLoadResult {
      try {
        return {std::make_shared<SidescanBagSession>(bag_uri), QString()};
      } catch (const std::exception & e) {
        return {nullptr, QString::fromStdString(e.what())};
      }
      }));
}

void SidescanViewerWindow::onLoadFinished()
{
  loading_ = false;
  const SidescanLoadResult result = load_watcher_.result();
  if (!result.session) {
    status_->setText("Open a bag to begin (File → Open Bag).");
    QMessageBox::critical(this, "Open bag failed", result.error);
    return;
  }
  session_ = result.session;

  // Boat track from every ping with a resolved pose (stamp-ordered).
  std::vector<QPointF> track;
  for (const auto & p : session_->pings()) {
    if (p.has_pose) {track.emplace_back(p.geometry.sensor_x, p.geometry.sensor_y);}
  }
  canvas_->setTrack(track);

  scrub_->setEnabled(true);
  scrub_->setValue(scrub_->maximum());  // start at the end of the track
  renderCurrentWindow();
  canvas_->resetView();

  status_->setText(QString(
      "%1 pings (%2 port, %3 stbd, %4 down) • %5 m track • alt: %6 • geo: %7")
    .arg(session_->pings().size())
    .arg(session_->channelCount(SidescanChannel::Port))
    .arg(session_->channelCount(SidescanChannel::Starboard))
    .arg(session_->channelCount(SidescanChannel::Down))
    .arg(session_->totalDistance(), 0, 'f', 1)
    .arg(session_->usedNadirDepth() ? "nadir_depth" : "estimator")
    .arg(session_->hasGeoReference() ? "yes" : "no"));
}

void SidescanViewerWindow::onScrubChanged()
{
  renderCurrentWindow();
}

void SidescanViewerWindow::onGridSpacingChanged(double metres)
{
  canvas_->setGridSpacing(metres);
}

void SidescanViewerWindow::onWindowLengthChanged(double metres)
{
  window_len_m_ = metres;
  renderCurrentWindow();
}

void SidescanViewerWindow::renderCurrentWindow()
{
  if (!session_) {return;}
  const double total = session_->totalDistance();
  const double frac = (scrub_->maximum() > 0) ?
    static_cast<double>(scrub_->value()) / scrub_->maximum() :
    0.0;
  const double head = frac * total;
  const DistanceWindow win = distance_window(head, window_len_m_, total);

  // Window pings: port + starboard, with a resolved pose and samples to paint.
  std::vector<const SidescanPing *> all = session_->window(win.lo, win.hi);
  std::vector<const SidescanPing *> paint;
  for (const auto * p : all) {
    if (!p->has_pose || p->amplitudes.empty()) {continue;}
    if (p->channel == SidescanChannel::Down) {continue;}
    paint.push_back(p);
  }
  // Stationary cap: keep the most recent pings (window list is distance-ordered).
  const int from = stationary_keep_from(static_cast<int>(paint.size()), max_window_pings_);
  paint.erase(paint.begin(), paint.begin() + from);

  if (paint.empty()) {
    canvas_->setCoverage(QImage(), 0.0, 0.0, resolution_m_);
    return;
  }

  // Bounding box of the painted swath: sensor positions padded by max ground range.
  double min_x = paint.front()->geometry.sensor_x;
  double max_x = min_x;
  double min_y = paint.front()->geometry.sensor_y;
  double max_y = min_y;
  double pad = 0.0;
  for (const auto * p : paint) {
    min_x = std::min(min_x, p->geometry.sensor_x);
    max_x = std::max(max_x, p->geometry.sensor_x);
    min_y = std::min(min_y, p->geometry.sensor_y);
    max_y = std::max(max_y, p->geometry.sensor_y);
    pad = std::max(pad, ping_max_range(*p));
  }
  min_x -= pad;
  max_x += pad;
  min_y -= pad;
  max_y += pad;

  // Size the raster; cap dimensions so a huge extent can't allocate unboundedly.
  int w = static_cast<int>(std::ceil((max_x - min_x) / resolution_m_)) + 1;
  int h = static_cast<int>(std::ceil((max_y - min_y) / resolution_m_)) + 1;
  constexpr int kMaxDim = 4000;
  w = std::clamp(w, 1, kMaxDim);
  h = std::clamp(h, 1, kMaxDim);

  CoverageRaster raster(min_x, min_y, resolution_m_, w, h);
  for (const auto * p : paint) {
    paint_ping(raster, p->geometry, p->amplitudes);
  }

  canvas_->setCoverage(render_coverage(raster), min_x, min_y, resolution_m_);

  status_->setText(QString("scrub %1 / %2 m • window [%3, %4] m • %5 pings painted")
    .arg(head, 0, 'f', 1)
    .arg(total, 0, 'f', 1)
    .arg(win.lo, 0, 'f', 1)
    .arg(win.hi, 0, 'f', 1)
    .arg(paint.size()));
}

}  // namespace marine_perception_tools
