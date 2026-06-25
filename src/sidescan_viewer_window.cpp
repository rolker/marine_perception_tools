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

#include <QAbstractItemView>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPointF>
#include <QCloseEvent>
#include <QProgressBar>
#include <QPushButton>
#include <QRectF>
#include <QSettings>
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
#include <optional>
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

// Build the uncorrected slant-range sidescan rows for a window as shared-lib
// WaterfallRows. Port and starboard pings are paired by along-track index into one
// centered row (combine_rows: port reversed left of nadir, starboard right of it),
// each side carrying its own slant range so the widget can scale/slant-correct, plus
// the sensor's map pose so a marked pixel inverts back to map coordinates. Returned
// oldest-first; the lib widget draws the newest row at the top (matching the live
// rqt plugin) as rows are appended. No georeferencing or slant->ground correction
// here — the widget does any correction at render time from range_max + altitude.
std::vector<marine_sonar_widgets::WaterfallRow> build_sidescan_rows(
  const std::vector<WindowPing> & pings)
{
  using marine_sonar_widgets::WaterfallRow;
  using marine_sonar_widgets::WorldPose;
  std::vector<const WindowPing *> port;
  std::vector<const WindowPing *> stbd;
  for (const auto & p : pings) {
    if (p.channel == SidescanChannel::Port) {
      port.push_back(&p);
    } else if (p.channel == SidescanChannel::Starboard) {
      stbd.push_back(&p);
    }
  }
  const std::size_t rows = std::max(port.size(), stbd.size());
  std::vector<WaterfallRow> out;
  out.reserve(rows);

  // Build one single-side row: raw samples + this side's slant range + altitude.
  // The data array starts at acoustic sample `sample0` (the sonar's near-range
  // gate), so element 0 sits at slant range sample0*mps, not 0. The lib widget maps
  // a row linearly from range 0, so prepend `sample0` empty (water-column) samples
  // to align index 0 with range 0 — keeping the display scale and the pixel->map
  // marking true to slant range (mirrors slant_range_at()'s sample0 offset, which
  // the retired SidescanWaterfall honored in its marking). The gate is clamped to
  // the sample count so a corrupt sample0 can't blow up the allocation.
  auto side_row = [](const WindowPing * p) -> std::optional<WaterfallRow> {
      if (p == nullptr) {return std::nullopt;}
      WaterfallRow w;
      const std::size_t gate = std::min<std::size_t>(
        p->geometry.sample0, p->amplitudes.size());
      w.intensities.reserve(gate + p->amplitudes.size());
      w.intensities.assign(gate, 0.0f);
      w.intensities.insert(
        w.intensities.end(), p->amplitudes.begin(), p->amplitudes.end());
      w.range_max = ping_max_range(*p);
      w.altitude = (p->geometry.altitude > 0.0) ? p->geometry.altitude : 0.0;
      return w;
    };

  for (std::size_t r = 0; r < rows; ++r) {
    const WindowPing * pp = (r < port.size()) ? port[r] : nullptr;
    const WindowPing * sp = (r < stbd.size()) ? stbd[r] : nullptr;
    auto combined = marine_sonar_widgets::combine_rows(side_row(pp), side_row(sp));
    if (!combined) {continue;}
    // Port and starboard share the platform pose; take whichever side this row has.
    const WindowPing * geom = (pp != nullptr) ? pp : sp;
    combined->altitude = (geom->geometry.altitude > 0.0) ? geom->geometry.altitude : 0.0;
    combined->world_pose = WorldPose{
      geom->geometry.sensor_x, geom->geometry.sensor_y, geom->geometry.yaw};
    out.push_back(std::move(*combined));
  }
  return out;
}

// Project one MBES ping's per-beam backscatter onto a uniform across-track axis so
// the pane is geometrically across-track (port on the left), the lib can draw metric
// range lines, and a marked pixel inverts exactly to a map point via the sensor pose.
// Each valid sounding's signed across-track ground distance (+ = port/left of the
// sensor heading) is binned per side; the row is laid out port-far -> nadir ->
// starboard-far with the side max ranges, matching the lib's WaterfallRow model.
marine_sonar_widgets::WaterfallRow build_mbes_backscatter_row(const MbesWindowPing & mp)
{
  using marine_sonar_widgets::WaterfallRow;
  using marine_sonar_widgets::WorldPose;
  WaterfallRow row;
  if (mp.world_soundings.empty()) {return row;}

  // Left-of-heading unit vector (ENU): the across-track axis. + = port (left).
  const double lhx = -std::sin(mp.heading);
  const double lhy = std::cos(mp.heading);
  struct AcrossSample {double across; float intensity;};
  std::vector<AcrossSample> samples;
  samples.reserve(mp.world_soundings.size());
  double port_max = 0.0;
  double stbd_max = 0.0;
  for (const auto & s : mp.world_soundings) {
    const double across = (s.x - mp.sensor_x) * lhx + (s.y - mp.sensor_y) * lhy;
    samples.push_back({across, s.intensity});
    if (across >= 0.0) {
      port_max = std::max(port_max, across);
    } else {
      stbd_max = std::max(stbd_max, -across);
    }
  }

  constexpr int kBins = 256;   // uniform across-track bins per side
  const double res_p = (port_max > 0.0) ? port_max / kBins : 0.0;
  const double res_s = (stbd_max > 0.0) ? stbd_max / kBins : 0.0;
  std::vector<double> port_sum(kBins, 0.0);
  std::vector<double> stbd_sum(kBins, 0.0);
  std::vector<int> port_n(kBins, 0);
  std::vector<int> stbd_n(kBins, 0);
  for (const auto & a : samples) {
    if (a.across >= 0.0 && res_p > 0.0) {
      const int b = std::min(kBins - 1, static_cast<int>(a.across / res_p));
      port_sum[b] += a.intensity;
      ++port_n[b];
    } else if (a.across < 0.0 && res_s > 0.0) {
      const int b = std::min(kBins - 1, static_cast<int>(-a.across / res_s));
      stbd_sum[b] += a.intensity;
      ++stbd_n[b];
    }
  }
  // Average per bin; fill empty bins by holding the previous (near->far) value so
  // sparse far-range beams don't read as black stripes. Leading empties stay 0.
  auto finish = [](const std::vector<double> & sum, const std::vector<int> & n) {
      std::vector<float> v(n.size(), 0.0f);
      float last = 0.0f;
      for (std::size_t i = 0; i < n.size(); ++i) {
        if (n[i] > 0) {
          v[i] = static_cast<float>(sum[i] / n[i]);
          last = v[i];
        } else {
          v[i] = last;
        }
      }
      return v;
    };
  const std::vector<float> port = finish(port_sum, port_n);   // near(0) -> far
  const std::vector<float> stbd = finish(stbd_sum, stbd_n);

  // Layout: port reversed (far..near) then starboard (near..far); nadir at the join.
  row.intensities.reserve((port_max > 0.0 ? kBins : 0) + (stbd_max > 0.0 ? kBins : 0));
  if (port_max > 0.0) {
    for (int i = kBins - 1; i >= 0; --i) {
      row.intensities.push_back(port[i]);
    }
  }
  row.nadir_index = row.intensities.size();
  if (stbd_max > 0.0) {
    for (int i = 0; i < kBins; ++i) {
      row.intensities.push_back(stbd[i]);
    }
  }
  row.range_max_port = port_max;
  row.range_max_stbd = stbd_max;
  row.range_max = std::max(port_max, stbd_max);
  if (mp.has_pose) {
    row.world_pose = WorldPose{mp.sensor_x, mp.sensor_y, mp.heading};
  }
  return row;
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

  // Uncorrected sidescan rows + the window's track centre (for the follow-the-
  // playhead recentre) — both from the same pings, so map and waterfall stay in
  // lockstep.
  out.sidescan_rows = build_sidescan_rows(paint);
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
    // Backscatter row: across-track-projected (metric, port on the left) so the pane
    // shows true across-track range lines and a marked pixel inverts to a map point.
    out.mbes_backscatter_rows.push_back(build_mbes_backscatter_row(mp));
  }

  // Down-channel water-column pings (raw) for the echogram, same window.
  out.down_images = session->readDownImages(win_lo, win_hi, max_pings);

  // Boat pose at the scrub head, for the 3D context arrow: the ping nearest `head`
  // gives the boat x/y/heading; place the arrow at the top of the cloud (near the
  // surface, above the seabed). Only valid when there are soundings to show.
  if (!out.mbes_soundings.empty()) {
    const WindowPing * nearest = &paint.front();
    double best = std::abs(paint.front().cumulative_distance_m - head);
    for (const auto & p : paint) {
      const double d = std::abs(p.cumulative_distance_m - head);
      if (d < best) {
        best = d;
        nearest = &p;
      }
    }
    double max_z = out.mbes_soundings.front().z;
    double min_z = max_z;
    for (const auto & s : out.mbes_soundings) {
      max_z = std::max(max_z, s.z);
      min_z = std::min(min_z, s.z);
    }
    // The boat is at the water surface — ABOVE the seabed by the water depth (the
    // nadir height-above-bottom). Lift the arrow above the cloud top by that depth
    // (or the cloud's vertical extent when altitude is unknown) so it reads as
    // floating on the surface, not sitting on the bottom. (G will use the true m3
    // sensor z instead of this estimate.)
    const double alt = nearest->geometry.altitude;
    const double lift = (alt > 0.0) ? alt : std::max(1.0, max_z - min_z);
    out.boat_x = nearest->geometry.sensor_x;
    out.boat_y = nearest->geometry.sensor_y;
    out.boat_z = max_z + lift;
    out.boat_heading = nearest->geometry.yaw;
    out.boat_valid = true;
  }

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

  // --- Data view widgets ---------------------------------------------------
  // Sidescan slant-range waterfall (shared-lib WaterfallWidget, GPU): inverts a
  // marked pixel to map coordinates from each row's pose; slant range (water column
  // kept) matches the raw display analysts read, with across-track range gridlines.
  waterfall_ = new marine_sonar_widgets::WaterfallWidget(this);
  waterfall_->set_ground_range(false);   // raw slant range, not slant->ground (palette set below)

  // MBES backscatter waterfall: one row per detection ping, the beam dB fan
  // across-track (beam-index axis, no metric range lines), newest at top.
  mbes_waterfall_ = new marine_sonar_widgets::WaterfallWidget(this);
  // Across-track-projected: keep slant range (we already supply ground/across-track
  // distances, so no further conversion) and draw across-track range lines.
  mbes_waterfall_->set_ground_range(false);
  mbes_waterfall_->set_range_lines(true);

  // Water-column echogram: the down-channel pings as a depth-vs-distance curtain.
  echogram_ = new marine_sonar_widgets::EchogramWidget(this);

  // MBES 3D point cloud: orbit view of the window's soundings (colour mode +
  // Z-exaggeration live).
  cloud_ = new PointCloudView(this);
  cloud_color_combo_ = new QComboBox(this);
  cloud_color_combo_->addItem("Depth");
  cloud_color_combo_->addItem("Backscatter");
  zexag_spin_ = new QDoubleSpinBox(this);
  zexag_spin_->setRange(1.0, 20.0);
  zexag_spin_->setSingleStep(0.5);
  zexag_spin_->setValue(1.0);          // no vertical exaggeration by default
  zexag_spin_->setPrefix("Z× ");
  point_size_spin_ = new QDoubleSpinBox(this);
  point_size_spin_->setRange(1.0, 12.0);
  point_size_spin_->setSingleStep(0.5);
  point_size_spin_->setValue(2.5);
  point_size_spin_->setPrefix("pt ");
  point_size_spin_->setToolTip("3D point size (pixels)");

  // Per-pane colormap selectors. Every pane offers the SAME full marine_colormap
  // palette set (the waterfalls/echogram via the lib's palette overload, the map +
  // 3D via marine_colormap directly), defaulting to bronze.
  auto make_cmap_combo = [this]() {
      auto * c = new QComboBox(this);
      for (const auto & name : marine_colormap::palette_names()) {
        c->addItem(QString::fromStdString(name));
      }
      if (const auto vi = marine_colormap::palette_index("bronze")) {
        c->setCurrentIndex(static_cast<int>(*vi));
      }
      return c;
    };
  sidescan_cmap_ = make_cmap_combo();
  mbes_cmap_ = make_cmap_combo();
  echo_cmap_ = make_cmap_combo();
  // Apply each combo's initial palette to its widget (combos don't fire on init).
  waterfall_->set_color_map(marine_colormap::palette(sidescan_cmap_->currentIndex()));
  mbes_waterfall_->set_color_map(marine_colormap::palette(mbes_cmap_->currentIndex()));
  echogram_->set_color_map(marine_colormap::palette(echo_cmap_->currentIndex()));
  cloud_palette_ = new QComboBox(this);
  for (const auto & name : marine_colormap::palette_names()) {
    cloud_palette_->addItem(QString::fromStdString(name));
  }
  if (const auto vi = marine_colormap::palette_index("bronze")) {
    cloud_palette_->setCurrentIndex(static_cast<int>(*vi));
    cloud_->setColorMap(static_cast<int>(*vi));
  }

  // Wrap a view in a titled panel with a small header row (title + per-pane controls).
  auto make_pane = [this](
    const QString & title, QWidget * view, const std::vector<QWidget *> & header) {
      auto * panel = new QWidget(this);
      auto * v = new QVBoxLayout(panel);
      v->setContentsMargins(2, 2, 2, 2);
      v->setSpacing(2);
      auto * hdr = new QHBoxLayout();
      hdr->addWidget(new QLabel(title, panel));
      hdr->addStretch(1);
      for (auto * w : header) {
        hdr->addWidget(w);
      }
      v->addLayout(hdr);
      v->addWidget(view, 1);
      return panel;
    };

  auto * ss_pane = make_pane("Sidescan", waterfall_, {sidescan_cmap_});
  auto * bs_pane = make_pane("MBES Backscatter", mbes_waterfall_, {mbes_cmap_});
  auto * wc_pane = make_pane("Water Column", echogram_, {echo_cmap_});
  auto * cloud_pane = make_pane(
    "MBES 3D", cloud_, {cloud_color_combo_, zexag_spin_, point_size_spin_, cloud_palette_});

  // 2x2 grid of the four sonar views, each pane independently resizable:
  //   sidescan waterfall (UL) | MBES backscatter (UR)
  //   MBES 3D            (LL) | water column     (LR)
  grid_top_split_ = new QSplitter(Qt::Horizontal, this);
  grid_top_split_->addWidget(ss_pane);
  grid_top_split_->addWidget(bs_pane);
  grid_bot_split_ = new QSplitter(Qt::Horizontal, this);
  grid_bot_split_->addWidget(cloud_pane);
  grid_bot_split_->addWidget(wc_pane);
  grid_split_ = new QSplitter(Qt::Vertical, this);
  grid_split_->addWidget(grid_top_split_);
  grid_split_->addWidget(grid_bot_split_);

  // Contacts list (left): one row per contact, click to recentre the map.
  contact_list_ = new QListWidget(this);
  auto * contacts_pane = make_pane("Contacts", contact_list_, {});

  // Left-to-right: contacts | map | 2x2 grid, all resizable.
  outer_split_ = new QSplitter(Qt::Horizontal, this);
  outer_split_->addWidget(contacts_pane);
  outer_split_->addWidget(canvas_);
  outer_split_->addWidget(grid_split_);
  outer_split_->setStretchFactor(0, 0);
  outer_split_->setStretchFactor(1, 3);
  outer_split_->setStretchFactor(2, 4);

  auto * central = new QWidget(this);
  auto * col = new QVBoxLayout(central);
  col->addWidget(outer_split_, 1);
  col->addWidget(controls);
  col->addWidget(status_);
  setCentralWidget(central);

  auto * file_menu = menuBar()->addMenu("&File");
  file_menu->addAction("&Open Bag…", this, &SidescanViewerWindow::onOpenBag);
  file_menu->addAction("&Fit View", this, [this]() {
      canvas_->resetView();
      cloud_->resetView();
    });
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
      mbes_waterfall_->setMarkMode(on);
    });
  connect(canvas_, &SidescanCanvas::boxMarked,
    this, &SidescanViewerWindow::onContactMarked);
  connect(waterfall_, &marine_sonar_widgets::WaterfallWidget::boxMarked,
    this, &SidescanViewerWindow::onContactMarked);
  connect(mbes_waterfall_, &marine_sonar_widgets::WaterfallWidget::boxMarked,
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
  connect(point_size_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, [this](double px) {cloud_->setPointSize(static_cast<float>(px));});

  // Per-pane colormaps: each repaints/recolours live, no re-render needed.
  connect(sidescan_cmap_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int i) {waterfall_->set_color_map(marine_colormap::palette(i));});
  connect(mbes_cmap_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int i) {mbes_waterfall_->set_color_map(marine_colormap::palette(i));});
  connect(echo_cmap_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int i) {echogram_->set_color_map(marine_colormap::palette(i));});
  connect(cloud_palette_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int i) {cloud_->setColorMap(i);});

  canvas_->setGridSpacing(grid_spin_->value());
  resize(1100, 760);

  // Watch app-wide key presses so scrub keys work regardless of which pane has
  // focus (handled in eventFilter; editing widgets keep their own key behaviour).
  qApp->installEventFilter(this);

  // Restore the operator's last window geometry + the resizable-pane splitter sizes.
  // No-op on first run.
  QSettings settings("UNH-CCOM", "sidescan_target_viewer");
  const QByteArray geom = settings.value("geometry").toByteArray();
  if (!geom.isEmpty()) {restoreGeometry(geom);}
  const auto restore_split = [&settings](QSplitter * s, const char * key) {
      if (s == nullptr) {return;}
      const QByteArray st = settings.value(key).toByteArray();
      if (!st.isEmpty()) {s->restoreState(st);}
    };
  restore_split(outer_split_, "split_outer");
  restore_split(grid_split_, "split_grid");
  restore_split(grid_top_split_, "split_grid_top");
  restore_split(grid_bot_split_, "split_grid_bot");
}

SidescanViewerWindow::~SidescanViewerWindow()
{
  // Don't let a worker outlive the widgets it would signal: wait for any in-flight
  // load/render to finish before the members tear down.
  if (load_watcher_.isRunning()) {load_watcher_.waitForFinished();}
  if (render_watcher_.isRunning()) {render_watcher_.waitForFinished();}
}

void SidescanViewerWindow::closeEvent(QCloseEvent * event)
{
  // Persist the window geometry + the resizable-pane splitter sizes so the
  // operator's arrangement survives a restart.
  QSettings settings("UNH-CCOM", "sidescan_target_viewer");
  settings.setValue("geometry", saveGeometry());
  if (outer_split_) {settings.setValue("split_outer", outer_split_->saveState());}
  if (grid_split_) {settings.setValue("split_grid", grid_split_->saveState());}
  if (grid_top_split_) {settings.setValue("split_grid_top", grid_top_split_->saveState());}
  if (grid_bot_split_) {settings.setValue("split_grid_bot", grid_bot_split_->saveState());}
  QMainWindow::closeEvent(event);
}

bool SidescanViewerWindow::eventFilter(QObject * obj, QEvent * event)
{
  if (event->type() == QEvent::KeyPress && scrub_ != nullptr && scrub_->isEnabled()) {
    // Don't steal navigation keys from widgets where they mean something else:
    // editing a spin box / combo / text field, navigating the contact list, or the
    // scrub slider itself (which already handles these keys when focused).
    QWidget * fw = QApplication::focusWidget();
    const bool editing =
      qobject_cast<QAbstractSpinBox *>(fw) != nullptr ||
      qobject_cast<QComboBox *>(fw) != nullptr ||
      qobject_cast<QLineEdit *>(fw) != nullptr ||
      qobject_cast<QAbstractItemView *>(fw) != nullptr ||
      qobject_cast<QAbstractSlider *>(fw) != nullptr;
    if (!editing) {
      auto * ke = static_cast<QKeyEvent *>(event);
      int v = scrub_->value();
      bool handled = true;
      switch (ke->key()) {
        case Qt::Key_Left: v -= scrub_->singleStep(); break;
        case Qt::Key_Right: v += scrub_->singleStep(); break;
        case Qt::Key_PageUp: v += scrub_->pageStep(); break;
        case Qt::Key_PageDown: v -= scrub_->pageStep(); break;
        case Qt::Key_Home: v = scrub_->minimum(); break;
        case Qt::Key_End: v = scrub_->maximum(); break;
        default: handled = false; break;
      }
      if (handled) {
        scrub_->setValue(std::clamp(v, scrub_->minimum(), scrub_->maximum()));
        return true;   // consumed
      }
    }
  }
  return QMainWindow::eventFilter(obj, event);
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
  cloud_->resetView();   // re-frame the 3D for the new bag (scrubs then keep the zoom)

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
  // Same contacts in the lib widget's map-frame box form, so the waterfall projects
  // each onto every pass that ensonified it.
  std::vector<marine_sonar_widgets::ContactBox> boxes;
  boxes.reserve(contact_store_.size());
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
    boxes.push_back(marine_sonar_widgets::ContactBox{
        p.x, p.y, c.shape.dimensions.x, c.shape.dimensions.y, m.id});
    contact_list_->addItem(QString("%1   %2 x %3 m   (%4, %5)")
      .arg(m.id)
      .arg(m.w, 0, 'f', 1).arg(m.h, 0, 'f', 1)
      .arg(p.x, 0, 'f', 1).arg(p.y, 0, 'f', 1));
  }
  canvas_->setContacts(markers);
  waterfall_->setContacts(boxes);
  mbes_waterfall_->setContacts(boxes);   // project contacts onto the backscatter pane too
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
  // Rebuild the sidescan waterfall for this window: size the scrollback to the
  // window so none of its rows are evicted (the lib widget's default 200-row history
  // is smaller than a dense window), then clear and append oldest-first so the newest
  // ping scrolls to the top (matching the live plugin).
  if (!r.sidescan_rows.empty()) {
    waterfall_->set_history(r.sidescan_rows.size());
  }
  waterfall_->clear();
  for (const auto & row : r.sidescan_rows) {
    waterfall_->add_row(row);
  }
  // The 3D cloud keeps its own palette (set in the ctor and via cloud_palette_);
  // setPoints recolours with that stored palette, so no per-render setColorMap here.
  cloud_->setPoints(r.mbes_soundings);
  cloud_->setBoat(r.boat_x, r.boat_y, r.boat_z, r.boat_heading, r.boat_valid);
  // Size the MBES backscatter scrollback to the window too (same reason as the
  // sidescan pane): the lib's default 200-row history is smaller than a dense
  // detections window, so without this the oldest MBES pings are evicted and the
  // backscatter pane falls out of lockstep with the other panes on the same scrub.
  if (!r.mbes_backscatter_rows.empty()) {
    mbes_waterfall_->set_history(r.mbes_backscatter_rows.size());
  }
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
