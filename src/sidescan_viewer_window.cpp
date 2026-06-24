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

#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPointF>
#include <QProgressBar>
#include <QPushButton>
#include <QRectF>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QString>
#include <QtConcurrent>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <utility>
#include <vector>

#include "contact_store.hpp"
#include "coverage_raster.hpp"
#include "distance_buffer_policy.hpp"
#include "marine_colormap/colormap.hpp"
#include "marine_colormap/palette.hpp"
#include "marine_colormap/transfer.hpp"
#include "marine_interfaces/msg/contact.hpp"
#include "marine_sonar_widgets/echogram_widget.hpp"
#include "marine_sonar_widgets/waterfall_widget.hpp"
#include "point_cloud_view.hpp"
#include "sidescan_canvas.hpp"
#include "sidescan_geometry.hpp"
#include "sidescan_waterfall.hpp"

namespace marine_perception_tools
{
namespace
{

// Maximum slant range (≈ far ground range) of a ping's last sample, used to pad
// the swath bounding box.
double ping_max_range(const WindowPing & p)
{
  if (p.amplitudes.empty() || p.geometry.metres_per_sample <= 0.0) {return 0.0;}
  return slant_range_at(p.amplitudes.size(), p.geometry.sample0, p.geometry.metres_per_sample);
}

// Auto colormap scale: low/high percentiles of the window's backscatter, so
// contrast adapts to the data. Histogram over [0, 1] (amplitudes are already
// normalized), O(n). Returns the full range if degenerate.
std::pair<float, float> auto_range(
  const std::vector<WindowPing> & pings, double lo_pct, double hi_pct)
{
  std::array<std::size_t, 256> hist{};
  std::size_t total = 0;
  for (const auto & p : pings) {
    for (float a : p.amplitudes) {
      hist[static_cast<std::size_t>(std::clamp(static_cast<int>(a * 255.0f), 0, 255))]++;
      ++total;
    }
  }
  if (total == 0) {return {0.0f, 1.0f};}
  const auto lo_count = static_cast<std::size_t>(lo_pct * total);
  const auto hi_count = static_cast<std::size_t>(hi_pct * total);
  int lo_bin = 0;
  int hi_bin = 255;
  std::size_t cum = 0;
  for (int i = 0; i < 256; ++i) {
    cum += hist[i]; if (cum >= lo_count) {
      lo_bin = i; break;
    }
  }
  cum = 0;
  for (int i = 0; i < 256; ++i) {
    cum += hist[i]; if (cum >= hi_count) {
      hi_bin = i; break;
    }
  }
  const float lo = lo_bin / 255.0f;
  const float hi = hi_bin / 255.0f;
  return (hi > lo) ? std::pair<float, float>{lo, hi} : std::pair<float, float>{0.0f, 1.0f};
}

// Map a normalized amplitude through the auto-range + baked colormap LUT.
QRgb lut_color(float amp, const std::vector<marine_colormap::Rgba8> & lut, float lo, float hi)
{
  float t = (hi > lo) ? (amp - lo) / (hi - lo) : amp;
  t = std::clamp(t, 0.0f, 1.0f);
  const auto & c = lut[static_cast<std::size_t>(std::clamp(
      static_cast<int>(t * (lut.size() - 1) + 0.5f), 0, static_cast<int>(lut.size() - 1)))];
  return qRgba(c.r, c.g, c.b, 255);
}

QImage render_coverage(
  const CoverageRaster & r, const std::vector<marine_colormap::Rgba8> & lut,
  float lo, float hi)
{
  QImage img(r.width(), r.height(), QImage::Format_ARGB32);
  img.fill(Qt::transparent);
  for (int row = 0; row < r.height(); ++row) {
    // Raster row 0 = south (min north); image row 0 = north edge (top). Write the
    // scanline directly (setPixel per cell is ~100x slower and was a scrub-time
    // bottleneck).
    const int img_row = r.height() - 1 - row;
    QRgb * line = reinterpret_cast<QRgb *>(img.scanLine(img_row));
    for (int col = 0; col < r.width(); ++col) {
      const float a = r.amplitudeAt(col, row);
      if (a < 0.0f) {continue;}  // uncovered -> transparent
      line[col] = lut_color(a, lut, lo, hi);
    }
  }
  return img;
}

// The uncorrected slant-range waterfall for a window: one row per ping cycle,
// port samples on the left (near range at centre, far range outward), starboard on
// the right. Port/starboard pings are paired by index in along-track order. Rows
// run newest-at-top (matching the live rqt plugin), so the channel lists are
// reversed and both align at the newest (top) edge. No georeferencing or slant
// correction — the raw stacked display.
QImage build_waterfall(
  const std::vector<WindowPing> & pings, WaterfallIndex & index,
  const std::vector<marine_colormap::Rgba8> & lut, float lo, float hi)
{
  index = WaterfallIndex{};
  std::vector<const WindowPing *> port;
  std::vector<const WindowPing *> stbd;
  for (const auto & p : pings) {
    if (p.channel == SidescanChannel::Port) {
      port.push_back(&p);
    } else if (p.channel == SidescanChannel::Starboard) {stbd.push_back(&p);}
  }
  std::reverse(port.begin(), port.end());   // newest first -> top row
  std::reverse(stbd.begin(), stbd.end());
  const int rows = static_cast<int>(std::max(port.size(), stbd.size()));
  if (rows == 0) {return QImage();}
  std::size_t pn = 0;
  std::size_t sn = 0;
  for (const auto * p : port) {
    pn = std::max(pn, p->amplitudes.size());
  }
  for (const auto * p : stbd) {
    sn = std::max(sn, p->amplitudes.size());
  }
  const int width = static_cast<int>(pn + sn);
  if (width == 0) {return QImage();}

  // Pixel->map index for waterfall marking (same newest-at-top row order).
  index.pn = static_cast<int>(pn);
  index.sn = static_cast<int>(sn);
  index.rows = rows;
  index.port_geo.reserve(port.size());
  index.stbd_geo.reserve(stbd.size());
  for (const auto * p : port) {
    index.port_geo.push_back(p->geometry);
  }
  for (const auto * p : stbd) {
    index.stbd_geo.push_back(p->geometry);
  }

  QImage img(width, rows, QImage::Format_ARGB32);
  img.fill(qRgba(0, 0, 0, 255));
  for (int r = 0; r < rows; ++r) {
    QRgb * line = reinterpret_cast<QRgb *>(img.scanLine(r));
    if (r < static_cast<int>(port.size())) {
      const auto & a = port[r]->amplitudes;
      for (std::size_t i = 0; i < a.size() && i < pn; ++i) {
        line[pn - 1 - i] = lut_color(a[i], lut, lo, hi);  // near->centre, far->left
      }
    }
    if (r < static_cast<int>(stbd.size())) {
      const auto & a = stbd[r]->amplitudes;
      for (std::size_t i = 0; i < a.size() && i < sn; ++i) {
        line[pn + i] = lut_color(a[i], lut, lo, hi);       // near->centre, far->right
      }
    }
  }
  return img;
}

// Build the coverage render for a distance window on a worker thread (readWindow +
// paint + rasterize + waterfall). Touches no widgets, so it is safe off the UI
// thread; the caller applies the result on the UI thread.
SidescanRenderResult render_window(
  std::shared_ptr<SidescanBagSession> session, double head, double total,
  double win_lo, double win_hi, int max_pings, double res, int palette_index)
{
  SidescanRenderResult out;
  out.res_m = res;
  out.head_m = head;
  out.total_m = total;
  out.win_lo = win_lo;
  out.win_hi = win_hi;
  out.ok = true;

  const std::vector<WindowPing> paint = session->readWindow(win_lo, win_hi, max_pings);
  out.npings = paint.size();
  if (paint.empty()) {return out;}  // ok, but a null image -> canvas clears

  // Shared colormap (marine_colormap, same as the rqt/rviz/CAMP apps) with an
  // auto contrast scale from this window's backscatter distribution.
  const auto [lo, hi] = auto_range(paint, 0.02, 0.98);
  const std::size_t n_pal = marine_colormap::palette_count();
  const std::size_t idx = (n_pal > 0) ?
    static_cast<std::size_t>(std::clamp(palette_index, 0, static_cast<int>(n_pal - 1))) :
    0;
  const auto lut = marine_colormap::bake_lut(
    marine_colormap::palette(idx), marine_colormap::TransferParams{}, 256);

  // Uncorrected waterfall + the window's track centre (for the follow-the-playhead
  // recentre) — both from the same pings, so map and waterfall stay in lockstep.
  out.waterfall = build_waterfall(paint, out.waterfall_index, lut, lo, hi);
  double cx = 0.0;
  double cy = 0.0;
  for (const auto & p : paint) {
    cx += p.geometry.sensor_x;
    cy += p.geometry.sensor_y;
  }
  out.center_x = cx / static_cast<double>(paint.size());
  out.center_y = cy / static_cast<double>(paint.size());
  out.has_center = true;

  // MBES soundings for the same window (world frame), flattened for the 3D view.
  // Shares the distance window so the cloud stays in lockstep with the scrub.
  const std::vector<MbesWindowPing> mwin = session->readMbesWindow(win_lo, win_hi, max_pings);
  std::size_t n_soundings = 0;
  for (const auto & mp : mwin) {
    n_soundings += mp.world_soundings.size();
  }
  out.mbes_soundings.reserve(n_soundings);
  out.mbes_backscatter_rows.reserve(mwin.size());
  for (const auto & mp : mwin) {
    out.mbes_soundings.insert(
      out.mbes_soundings.end(), mp.world_soundings.begin(), mp.world_soundings.end());
    // Backscatter waterfall row: the per-beam dB fan, centred (beam index is the
    // across-track axis; non-metric so no slant/ground range lines).
    marine_sonar_widgets::WaterfallRow row;
    row.intensities = mp.intensities;
    row.nadir_index = mp.intensities.size() / 2;
    out.mbes_backscatter_rows.push_back(std::move(row));
  }

  // Down-channel water-column pings (raw) for the echogram, same window.
  out.down_images = session->readDownImages(win_lo, win_hi, max_pings);

  double min_x = paint.front().geometry.sensor_x;
  double max_x = min_x;
  double min_y = paint.front().geometry.sensor_y;
  double max_y = min_y;
  double pad = 0.0;
  for (const auto & p : paint) {
    min_x = std::min(min_x, p.geometry.sensor_x);
    max_x = std::max(max_x, p.geometry.sensor_x);
    min_y = std::min(min_y, p.geometry.sensor_y);
    max_y = std::max(max_y, p.geometry.sensor_y);
    pad = std::max(pad, ping_max_range(p));
  }
  min_x -= pad;
  max_x += pad;
  min_y -= pad;
  max_y += pad;

  int w = static_cast<int>(std::ceil((max_x - min_x) / res)) + 1;
  int h = static_cast<int>(std::ceil((max_y - min_y) / res)) + 1;
  constexpr int kMaxDim = 4000;
  w = std::clamp(w, 1, kMaxDim);
  h = std::clamp(h, 1, kMaxDim);

  CoverageRaster raster(min_x, min_y, res, w, h);
  for (const auto & p : paint) {
    paint_ping(raster, p.geometry, p.amplitudes);
  }

  out.image = render_coverage(raster, lut, lo, hi);
  out.origin_x = min_x;
  out.origin_y = min_y;
  return out;
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

  max_pings_spin_ = new QSpinBox(this);
  max_pings_spin_->setRange(50, 50000);
  max_pings_spin_->setValue(max_window_pings_);
  max_pings_spin_->setSingleStep(10);
  max_pings_spin_->setSuffix(" pings");
  max_pings_spin_->setToolTip(
    "Max pings rendered per window (stationary cap). Also sets the map raster "
    "resolution = window / this, so raising it for a slow (dense-ping) survey "
    "both renders more pings and refines the map.");

  status_ = new QLabel("Open a bag to begin (File → Open Bag).", this);

  // Indeterminate "busy" bar shown only while a bag loads off-thread.
  progress_ = new QProgressBar(this);
  progress_->setRange(0, 0);
  progress_->setTextVisible(false);
  progress_->setMaximumWidth(160);
  progress_->setVisible(false);

  auto * controls = new QWidget(this);
  auto * crow = new QHBoxLayout(controls);
  crow->addWidget(new QLabel("Distance:", this));
  crow->addWidget(scrub_, 1);
  crow->addWidget(new QLabel("Grid:", this));
  crow->addWidget(grid_spin_);
  crow->addWidget(new QLabel("Window:", this));
  crow->addWidget(window_spin_);
  crow->addWidget(new QLabel("Max:", this));
  crow->addWidget(max_pings_spin_);
  mark_button_ = new QPushButton("Mark contact", this);
  mark_button_->setCheckable(true);
  mark_button_->setToolTip("Toggle marking: drag a box around a target on the map.");
  crow->addWidget(mark_button_);

  // Shared marine_colormap palette selector (same palettes as the rqt/rviz apps).
  palette_combo_ = new QComboBox(this);
  for (const auto & name : marine_colormap::palette_names()) {
    palette_combo_->addItem(QString::fromStdString(name));
  }
  if (const auto vi = marine_colormap::palette_index("bronze")) {
    palette_combo_->setCurrentIndex(static_cast<int>(*vi));
  }
  crow->addWidget(new QLabel("Palette:", this));
  crow->addWidget(palette_combo_);

  crow->addWidget(progress_);

  // Geo map (left) beside the uncorrected waterfall (right), user-resizable.
  waterfall_ = new SidescanWaterfall(this);
  auto * split = new QSplitter(Qt::Horizontal, this);
  split->addWidget(canvas_);
  split->addWidget(waterfall_);
  split->setStretchFactor(0, 3);
  split->setStretchFactor(1, 1);

  auto * central = new QWidget(this);
  auto * col = new QVBoxLayout(central);
  col->addWidget(split, 1);
  col->addWidget(controls);
  col->addWidget(status_);
  setCentralWidget(central);

  // Target-list dock (right): one row per contact, click to recentre the map.
  contact_list_ = new QListWidget(this);
  auto * dock = new QDockWidget("Contacts", this);
  dock->setWidget(contact_list_);
  addDockWidget(Qt::RightDockWidgetArea, dock);

  // MBES 3D point-cloud dock (right): orbit view of the window's soundings, with
  // colour-mode + Z-exaggeration controls above it.
  cloud_ = new PointCloudView(this);
  cloud_color_combo_ = new QComboBox(this);
  cloud_color_combo_->addItem("Depth");
  cloud_color_combo_->addItem("Backscatter");
  zexag_spin_ = new QDoubleSpinBox(this);
  zexag_spin_->setRange(1.0, 20.0);
  zexag_spin_->setSingleStep(0.5);
  zexag_spin_->setValue(3.0);
  zexag_spin_->setPrefix("Z× ");
  auto * cloud_panel = new QWidget(this);
  auto * cloud_col = new QVBoxLayout(cloud_panel);
  auto * cloud_ctrls = new QHBoxLayout();
  cloud_ctrls->addWidget(new QLabel("Colour:", this));
  cloud_ctrls->addWidget(cloud_color_combo_);
  cloud_ctrls->addWidget(zexag_spin_);
  cloud_ctrls->addStretch(1);
  cloud_col->addLayout(cloud_ctrls);
  cloud_col->addWidget(cloud_, 1);
  auto * cloud_dock = new QDockWidget("MBES 3D", this);
  cloud_dock->setWidget(cloud_panel);
  addDockWidget(Qt::RightDockWidgetArea, cloud_dock);

  // MBES backscatter waterfall dock (shared lib WaterfallWidget): one row per
  // detection ping, the 224/234-beam dB fan across-track, newest at top.
  mbes_waterfall_ = new marine_sonar_widgets::WaterfallWidget(this);
  mbes_waterfall_->set_color_map(marine_sonar_widgets::ColorMapType::Bronze);
  mbes_waterfall_->set_range_lines(false);   // beam-index axis, not metric range
  auto * mbes_wf_dock = new QDockWidget("MBES Backscatter", this);
  mbes_wf_dock->setWidget(mbes_waterfall_);
  addDockWidget(Qt::RightDockWidgetArea, mbes_wf_dock);

  // Water-column echogram dock (shared lib EchogramWidget): the down-channel pings
  // of the current window as a depth-vs-distance curtain.
  echogram_ = new marine_sonar_widgets::EchogramWidget(this);
  auto * echo_dock = new QDockWidget("Water Column", this);
  echo_dock->setWidget(echogram_);
  addDockWidget(Qt::RightDockWidgetArea, echo_dock);

  auto * file_menu = menuBar()->addMenu("&File");
  file_menu->addAction("&Open Bag…", this, &SidescanViewerWindow::onOpenBag);
  file_menu->addAction("&Fit View", this, [this]() {canvas_->resetView();});
  file_menu->addSeparator();
  file_menu->addAction("&Load Contacts…", this, &SidescanViewerWindow::onLoadContacts);
  file_menu->addAction("&Save Contacts…", this, &SidescanViewerWindow::onSaveContacts);
  file_menu->addSeparator();
  file_menu->addAction("E&xit", this, &QWidget::close);

  connect(&load_watcher_, &QFutureWatcher<SidescanLoadResult>::finished,
    this, &SidescanViewerWindow::onLoadFinished);
  connect(&render_watcher_, &QFutureWatcher<SidescanRenderResult>::finished,
    this, &SidescanViewerWindow::onRenderFinished);
  connect(scrub_, &QSlider::valueChanged, this, &SidescanViewerWindow::onScrubChanged);
  connect(grid_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, &SidescanViewerWindow::onGridSpacingChanged);
  connect(window_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, &SidescanViewerWindow::onWindowLengthChanged);
  connect(max_pings_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
    this, &SidescanViewerWindow::onMaxPingsChanged);
  connect(palette_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) {requestRender();});
  connect(mark_button_, &QPushButton::toggled, this, [this](bool on) {
      canvas_->setMarkMode(on);
      waterfall_->setMarkMode(on);
    });
  connect(canvas_, &SidescanCanvas::boxMarked,
    this, &SidescanViewerWindow::onContactMarked);
  connect(waterfall_, &SidescanWaterfall::boxMarked,
    this, &SidescanViewerWindow::onContactMarked);
  connect(contact_list_, &QListWidget::currentRowChanged, this, [this](int row) {
      if (row >= 0 && row < static_cast<int>(contact_store_.contacts().size())) {
        const auto & k = contact_store_.contacts()[row].kinematics.pose.pose.position;
        canvas_->setCenter(k.x, k.y);
      }
    });
  connect(cloud_color_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int i) {
      cloud_->setColorMode(
        i == 1 ? PointCloudView::ColorMode::Backscatter : PointCloudView::ColorMode::Depth);
    });
  connect(zexag_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, [this](double z) {cloud_->setZExaggeration(static_cast<float>(z));});

  canvas_->setGridSpacing(grid_spin_->value());
  resize(1100, 760);
}

SidescanViewerWindow::~SidescanViewerWindow()
{
  // Don't let a worker outlive the widgets it would signal: wait for any in-flight
  // load/render to finish before the members tear down.
  if (load_watcher_.isRunning()) {load_watcher_.waitForFinished();}
  if (render_watcher_.isRunning()) {render_watcher_.waitForFinished();}
}

void SidescanViewerWindow::onOpenBag()
{
  const QString dir = QFileDialog::getExistingDirectory(this, "Open ROS 2 bag directory");
  if (!dir.isEmpty()) {openBag(dir.toStdString());}
}

void SidescanViewerWindow::openBag(const std::string & bag_uri)
{
  // Opening a new bag while one is still loading replaces it: setFuture() below
  // makes the watcher track only the newest load, so the stale one's result is
  // dropped on arrival (its worker runs to completion harmlessly).
  loading_ = true;
  scrub_->setEnabled(false);
  progress_->setVisible(true);
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
  progress_->setVisible(false);
  const SidescanLoadResult result = load_watcher_.result();
  if (!result.session) {
    status_->setText("Open a bag to begin (File → Open Bag).");
    QMessageBox::critical(this, "Open bag failed", result.error);
    return;
  }
  session_ = result.session;
  ++session_epoch_;   // any render in flight for the previous bag is now stale

  // Boat track from every ping with a resolved pose (stamp-ordered).
  std::vector<QPointF> track;
  for (const auto & p : session_->pings()) {
    if (p.has_pose) {track.emplace_back(p.geometry.sensor_x, p.geometry.sensor_y);}
  }
  canvas_->setTrack(track);
  canvas_->resetView();  // fit to the track now; coverage fills in asynchronously

  // Slider units are metres of along-track distance, so the arrow-key step can be
  // an exact fraction of the window.
  scrub_->setEnabled(true);
  scrub_->blockSignals(true);
  scrub_->setRange(0, std::max(1, static_cast<int>(std::lround(session_->totalDistance()))));
  scrub_->setValue(0);  // start at the beginning of the track
  scrub_->blockSignals(false);
  updateScrubStep();
  requestRender();

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
  requestRender();
}

void SidescanViewerWindow::onGridSpacingChanged(double metres)
{
  canvas_->setGridSpacing(metres);
}

void SidescanViewerWindow::onWindowLengthChanged(double metres)
{
  window_len_m_ = metres;
  updateScrubStep();
  requestRender();
}

void SidescanViewerWindow::onMaxPingsChanged(int max_pings)
{
  max_window_pings_ = max_pings;
  requestRender();
}

void SidescanViewerWindow::onContactMarked(const QRectF & map_rect)
{
  std::vector<MapPoint> pts{
    {map_rect.left(), map_rect.top()},
    {map_rect.right(), map_rect.bottom()}};
  const QString id = QString("T-%1").arg(++contact_counter_, 3, 10, QChar('0'));
  const double head = std::clamp(static_cast<double>(scrub_->value()), 0.0,
    session_ ? session_->totalDistance() : 0.0);
  const double stamp_s = session_ ? session_->timeAtDistance(head) : 0.0;
  auto contact = make_box_contact(pts, id.toStdString(), "sidescan", "bizzy/map", stamp_s);

  // Resolve geo_pose (lat/lon) from the contact's map-frame centroid when the bag
  // carried an earth->map reference; leave it unresolved (NaN) otherwise.
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  if (session_ && session_->mapToGeo(
      contact.kinematics.pose.pose.position.x,
      contact.kinematics.pose.pose.position.y, lat, lon, alt))
  {
    contact.geo_pose.position.latitude = lat;
    contact.geo_pose.position.longitude = lon;
    contact.geo_pose.position.altitude = alt;
    // Orientation unknown for a point pick: all-zero quaternion per Contact.msg.
    contact.geo_pose.orientation.x = 0.0;
    contact.geo_pose.orientation.y = 0.0;
    contact.geo_pose.orientation.z = 0.0;
    contact.geo_pose.orientation.w = 0.0;
  }
  contact_store_.add(contact);
  refreshContacts();
}

void SidescanViewerWindow::onSaveContacts()
{
  const QString path = QFileDialog::getSaveFileName(
    this, "Save contacts", QString(), "Contact store (*.cdr)");
  if (path.isEmpty()) {return;}
  if (!contact_store_.save(path.toStdString())) {
    QMessageBox::warning(this, "Save failed", "Could not write " + path);
  }
}

void SidescanViewerWindow::onLoadContacts()
{
  const QString path = QFileDialog::getOpenFileName(
    this, "Load contacts", QString(), "Contact store (*.cdr)");
  if (path.isEmpty()) {return;}
  if (!contact_store_.load(path.toStdString())) {
    QMessageBox::warning(this, "Load failed", "Could not read " + path);
    return;
  }
  contact_counter_ = std::max(contact_counter_, static_cast<int>(contact_store_.size()));
  refreshContacts();
}

void SidescanViewerWindow::refreshContacts()
{
  QVector<ContactMarker> markers;
  markers.reserve(static_cast<int>(contact_store_.size()));
  contact_list_->clear();
  for (const auto & c : contact_store_.contacts()) {
    const auto & p = c.kinematics.pose.pose.position;
    ContactMarker m;
    m.x = p.x;
    m.y = p.y;
    m.w = c.shape.dimensions.x;
    m.h = c.shape.dimensions.y;
    m.id = QString::fromStdString(c.id);
    markers.push_back(m);
    contact_list_->addItem(QString("%1   %2 x %3 m   (%4, %5)")
      .arg(m.id)
      .arg(m.w, 0, 'f', 1).arg(m.h, 0, 'f', 1)
      .arg(p.x, 0, 'f', 1).arg(p.y, 0, 'f', 1));
  }
  canvas_->setContacts(markers);
  waterfall_->setContacts(markers);
}

void SidescanViewerWindow::updateScrubStep()
{
  // Slider units are metres: arrow keys step 20% of the window, PageUp/Down a full
  // window.
  scrub_->setSingleStep(std::max(1, static_cast<int>(std::lround(0.2 * window_len_m_))));
  scrub_->setPageStep(std::max(1, static_cast<int>(std::lround(window_len_m_))));
}

void SidescanViewerWindow::requestRender()
{
  if (!session_) {return;}
  // Coalesce: if a render is already running, flag a pending one and re-launch on
  // finish with the latest scrub position (so a drag never queues a backlog).
  if (rendering_) {
    render_pending_ = true;
    return;
  }
  const double total = session_->totalDistance();
  const double head = std::clamp(static_cast<double>(scrub_->value()), 0.0, total);
  const DistanceWindow win = distance_window(head, window_len_m_, total);

  rendering_ = true;
  auto session = session_;  // keep alive for the worker
  const int max_pings = max_window_pings_;
  const double res = window_len_m_ / static_cast<double>(std::max(1, max_window_pings_));
  const int palette = palette_combo_ ? palette_combo_->currentIndex() : 0;
  const uint64_t epoch = session_epoch_;
  render_watcher_.setFuture(QtConcurrent::run(
      [session, head, total, win, max_pings, res, palette, epoch]() {
        SidescanRenderResult r;
        try {
          r = render_window(
            session, head, total, win.lo, win.hi, max_pings, res, palette);
        } catch (const std::exception &) {
          r.ok = false;   // e.g. the bag became unreadable mid-session
        }
        r.epoch = epoch;
        return r;
      }));
}

void SidescanViewerWindow::onRenderFinished()
{
  rendering_ = false;
  const SidescanRenderResult r = render_watcher_.result();

  // Drop a render computed for a previous bag (epoch mismatch) so it never flashes
  // stale coverage or leaves the waterfall index on the wrong geometry.
  if (r.epoch != session_epoch_) {
    if (render_pending_) {
      render_pending_ = false;
      requestRender();
    }
    return;
  }

  if (!r.ok) {
    // The render worker hit an exception (e.g. the bag became unreadable); keep the
    // last good view rather than crashing or clearing.
    status_->setText("Render failed (bag unreadable?) — showing the last view.");
    if (render_pending_) {
      render_pending_ = false;
      requestRender();
    }
    return;
  }

  canvas_->setCoverage(r.image, r.origin_x, r.origin_y, r.res_m);
  waterfall_->setImage(r.waterfall);
  waterfall_->setIndex(r.waterfall_index);
  cloud_->setColorMap(palette_combo_->currentIndex());
  cloud_->setPoints(r.mbes_soundings);
  mbes_waterfall_->clear();
  for (const auto & row : r.mbes_backscatter_rows) {
    mbes_waterfall_->add_row(row);
  }
  // Echogram has no clear(); sizing history to the window count makes the new
  // pings evict the previous window's, so the curtain shows just this window.
  if (!r.down_images.empty()) {
    echogram_->setHistory(static_cast<int>(r.down_images.size()));
    echogram_->addPings(r.down_images);
  }
  if (r.has_center) {canvas_->setCenter(r.center_x, r.center_y);}
  status_->setText(QString("scrub %1 / %2 m • window [%3, %4] m • %5 pings painted")
    .arg(r.head_m, 0, 'f', 1)
    .arg(r.total_m, 0, 'f', 1)
    .arg(r.win_lo, 0, 'f', 1)
    .arg(r.win_hi, 0, 'f', 1)
    .arg(r.npings));

  // A scrub arrived while we were rendering — render once more with the latest.
  if (render_pending_) {
    render_pending_ = false;
    requestRender();
  }
}

}  // namespace marine_perception_tools
