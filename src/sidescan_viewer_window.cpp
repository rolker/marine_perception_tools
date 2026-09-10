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
#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QClipboard>
#include <QColor>
#include <QComboBox>
#include <QStandardItemModel>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDebug>
#include <QDir>
#include <QElapsedTimer>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFormLayout>
#include <QFileDialog>
#include <QFileInfo>
#include <QFont>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QImage>
#include <QKeyEvent>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPixmap>
#include <QPointF>
#include <QCloseEvent>
#include <QProgressBar>
#include <QPushButton>
#include <QRectF>
#include <QSettings>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QString>
#include <QStringList>
#include <QtConcurrent>
#include <QTimeZone>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "basemap_contrast.hpp"
#include "basemap_lod.hpp"
#include "cube_bathymetry/angular_response_curve.h"
#include "cube_export.hpp"
#include "sidescan_drape_loader.hpp"
#include "marine_autonomy/gggs.h"
#include "marine_contacts/contact_store.hpp"
#include "clamped_entry_spin.hpp"
#include "coastline_data.hpp"
#include "coverage_raster.hpp"
#include "distance_buffer_policy.hpp"
#include "map_geo_anchor.hpp"
#include "tf_lift.hpp"   // rotate_by_quat (the export anchor probe)
#include "worker_cancel.hpp"   // supersede_token
#include "world_layout.hpp"
#include "marine_colormap/colormap.hpp"
#include "marine_colormap/palette.hpp"
#include "marine_colormap/transfer.hpp"
#include "marine_interfaces/msg/contact.hpp"
#include "marine_sonar_widgets/echogram_widget.hpp"
#include "marine_sonar_widgets/waterfall_widget.hpp"
#include "marine_tiled_raster_store/tile_io.hpp"
#include "nav_track_lookup.hpp"
#include "pass_coalesce.hpp"
#include "point_cloud_view.hpp"
#include "session_index_io.hpp"
#include "sidescan_canvas.hpp"
#include "sidescan_geometry.hpp"
#include "sounding_uncertainty.hpp"

namespace marine_perception_tools
{
namespace
{

// --- shared colour vocabulary (#36) -----------------------------------------

// The channel a selector row stands for (stored as the item's user data, so
// the two selectors never depend on each other's row ORDER).
ColorChannel channel_at(const QComboBox * combo, int row)
{
  if (!combo || row < 0 || row >= combo->count()) {
    return ColorChannel::Depth;
  }
  return static_cast<ColorChannel>(combo->itemData(row).toInt());
}

// Enable or grey out one row. `reason` is nullptr when the channel is
// available; otherwise it greys the row AND becomes its tooltip — a channel a
// layer cannot carry must say why, never quietly vanish from the list.
void set_channel_available(QComboBox * combo, int row, const char * reason)
{
  if (!combo) {return;}
  if (auto * model = qobject_cast<QStandardItemModel *>(combo->model())) {
    if (auto * item = model->item(row)) {
      item->setEnabled(reason == nullptr);
    }
  }
  combo->setItemData(
    row, reason ? QString::fromUtf8(reason) : QString(), Qt::ToolTipRole);
}

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
// `cancel` (optional) is polled between the three bag re-reads (each of which
// polls it per message itself) and per ping while the coverage is painted —
// the units a window render is made of (#44). A cancelled render is marked
// `cancelled` and delivers nothing: a window painted from half its pings is
// not a coarser view, it is a wrong one.
SidescanRenderResult render_window(
  std::shared_ptr<SidescanBagSession> session, double head, double total,
  double win_lo, double win_hi, int max_pings, double res, int palette_index,
  std::optional<std::pair<float, float>> manual_range = std::nullopt,
  const std::shared_ptr<std::atomic<bool>> & cancel = {})
{
  const auto stop = [&cancel]() {
      return cancel && cancel->load(std::memory_order_relaxed);
    };
  SidescanRenderResult out;
  out.res_m = res;
  out.head_m = head;
  out.total_m = total;
  out.win_lo = win_lo;
  out.win_hi = win_hi;
  out.ok = true;

  const std::vector<WindowPing> paint =
    session->readWindow(win_lo, win_hi, max_pings, false, cancel);
  if (stop()) {
    out.ok = false;
    out.cancelled = true;
    return out;
  }
  out.npings = paint.size();
  if (paint.empty()) {return out;}  // ok, but a null image -> canvas clears

  // Shared colormap (marine_colormap, same as the rqt/rviz/CAMP apps) with an
  // auto contrast scale from this window's backscatter distribution — or the
  // operator's manual range (#26), shared with the sidescan waterfall so the
  // map overlay and waterfall read identically.
  const auto [lo, hi] = manual_range ? *manual_range : auto_range(paint, 0.02, 0.98);
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
  const std::vector<MbesWindowPing> mwin =
    session->readMbesWindow(win_lo, win_hi, max_pings, cancel);
  if (stop()) {
    out.ok = false;
    out.cancelled = true;
    return out;
  }
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
  out.down_images = session->readDownImages(win_lo, win_hi, max_pings, cancel);
  if (stop()) {
    out.ok = false;
    out.cancelled = true;
    return out;
  }

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
    if (stop()) {
      out.ok = false;
      out.cancelled = true;
      return out;
    }
    paint_ping(raster, p.geometry, p.amplitudes);
  }

  out.image = render_coverage(raster, lut, lo, hi);
  out.origin_x = min_x;
  out.origin_y = min_y;
  return out;
}

}  // namespace

void SidescanViewerWindow::setupCloudControls()
{
  cloud_color_combo_ = new QComboBox(this);
  cloud_color_combo_->setObjectName("cloud_color_combo");
  // The SAME vocabulary the surface shade offers (#36). Pass is an ordinary
  // entry here — selectable and deselectable — never a mode that takes the
  // control away; the entries the soundings cannot carry are greyed with
  // their reason, so the two lists read as one.
  populateColorVocabulary(cloud_color_combo_);
  cloud_color_combo_->setToolTip(
    "Point colouring. Greyed entries name a channel a sounding does not "
    "carry — the reason is on the entry.");
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

  // The cloud's own palette, independent of the surface's (the CUBE row has
  // its own) so cloud and surface can contrast. Defaults to bronze, applied
  // to the view here because a combo does not fire on construction.
  cloud_palette_ = new QComboBox(this);
  for (const auto & name : marine_colormap::palette_names()) {
    cloud_palette_->addItem(QString::fromStdString(name));
  }
  if (const auto vi = marine_colormap::palette_index("bronze")) {
    cloud_palette_->setCurrentIndex(static_cast<int>(*vi));
    cloud_->setColorMap(static_cast<int>(*vi));
  }
}

void SidescanViewerWindow::setupRangeControls()
{
  // Per-pane colour-range controls (#26): "auto" (default) or manual lo/hi
  // spin boxes in the pane's native units; the spins enable when auto is off.
  const auto make_range = [this](
    RangeControls & rc, double min, double max, double step, int decimals,
    double init_lo, double init_hi, const QString & tip) {
      rc.auto_check = new QCheckBox("auto", this);
      rc.auto_check->setChecked(true);
      rc.auto_check->setToolTip(tip);
      rc.lo = new QDoubleSpinBox(this);
      rc.hi = new QDoubleSpinBox(this);
      for (auto * s : {rc.lo, rc.hi}) {
        s->setRange(min, max);
        s->setDecimals(decimals);
        s->setSingleStep(step);
        s->setEnabled(false);
        s->setToolTip(tip);
        s->setKeyboardTracking(false);   // apply on commit, not per keystroke
      }
      rc.lo->setValue(init_lo);
      rc.hi->setValue(init_hi);
    };
  make_range(
    ss_range_, 0.0, 1.0, 0.02, 3, 0.0, 1.0,
    "Sidescan colour range (normalized amplitude); also scales the map's "
    "coverage overlay");
  make_range(
    bs_range_, 0.0, 1.0, 0.02, 3, 0.0, 1.0,
    "MBES backscatter colour range (normalized amplitude)");
  make_range(
    wc_range_, 0.0, 1.0, 0.02, 3, 0.0, 1.0,
    "Water-column black/white points within the buffered data extent");
  make_range(
    cloud_range_, -12000.0, 12000.0, 1.0, 1, -50.0, 0.0,
    "3D cloud colour range in the active scalar's units (depth m / intensity)");
  make_range(
    map_range_, -12000.0, 12000.0, 1.0, 1, -50.0, 0.0,
    "Basemap contrast range in layer units (auto = robust percentile scale)");

  // Range-control wiring (#26): one applier per pane, fired by the auto
  // toggle (which also gates the spins) and by either spin commit.
  const auto wire_range = [this](RangeControls & rc, std::function<void()> apply) {
      RangeControls * p = &rc;   // the member outlives every connection
      connect(p->auto_check, &QCheckBox::toggled, this, [p, apply](bool on) {
          p->lo->setEnabled(!on);
          p->hi->setEnabled(!on);
          apply();
        });
      connect(p->lo, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this, [p, apply](double) {if (!p->auto_check->isChecked()) {apply();}});
      connect(p->hi, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
        this, [p, apply](double) {if (!p->auto_check->isChecked()) {apply();}});
    };
  wire_range(ss_range_, [this]() {
      if (ss_range_.auto_check->isChecked()) {
        waterfall_->set_auto_range(true);
      } else {
        waterfall_->set_auto_range(false);
        waterfall_->set_manual_range(
          static_cast<float>(ss_range_.lo->value()),
          static_cast<float>(ss_range_.hi->value()));
      }
      requestRender();   // the map coverage overlay follows the same range
    });
  wire_range(bs_range_, [this]() {
      if (bs_range_.auto_check->isChecked()) {
        mbes_waterfall_->set_auto_range(true);
      } else {
        mbes_waterfall_->set_auto_range(false);
        mbes_waterfall_->set_manual_range(
          static_cast<float>(bs_range_.lo->value()),
          static_cast<float>(bs_range_.hi->value()));
      }
    });
  wire_range(wc_range_, [this]() {
      const bool a = wc_range_.auto_check->isChecked();
      echogram_->setAutoRange(a);
      if (!a) {
        echogram_->setBlackPoint(static_cast<float>(wc_range_.lo->value()));
        echogram_->setWhitePoint(static_cast<float>(wc_range_.hi->value()));
      }
    });
  wire_range(cloud_range_, [this]() {
      if (cloud_range_.auto_check->isChecked()) {
        cloud_->setScalarRange(std::nullopt);
      } else {
        cloud_->setScalarRange(std::pair<float, float>(
            static_cast<float>(cloud_range_.lo->value()),
            static_cast<float>(cloud_range_.hi->value())));
      }
    });
  wire_range(map_range_, [this]() {
      if (!basemap_lod_) {
        return;
      }
      if (map_range_.auto_check->isChecked()) {
        basemap_lod_->setRangeOverride(std::nullopt);
      } else {
        basemap_lod_->setRangeOverride(std::pair<double, double>(
            map_range_.lo->value(), map_range_.hi->value()));
      }
    });
}

void SidescanViewerWindow::setupCubeLab(QWidget * cloud_pane)
{
  // CUBE-lab controls (#27) in their own row under the cloud pane's header:
  // cell size, IHO order, the explicit Run trigger (CUBE is expensive — no
  // auto-runs), and the surface's display controls.
  // The floor is one display unit, not a judgement about what is worth
  // gridding (#42). It used to be 0.02 m, which silently swallowed the 0.01 m
  // the operator was deliberately testing with: Qt reverts out-of-range text
  // to the last valid value on focus-out — i.e. on the click of Run CUBE.
  // Nothing here needs to protect memory; the max-nodes pre-flight in
  // runCubeLab() bounds the allocation and offers a one-shot override. What is
  // left to protect is the arithmetic, which divides the box by the cell size,
  // so the floor is simply the smallest number the box can display (1 mm at
  // three decimals) — two orders of magnitude below any real beam footprint,
  // and self-consistent in that every enterable value is also a showable one.
  auto * cube_cell_spin = new ClampedEntryDoubleSpinBox(this);
  cube_cell_spin_ = cube_cell_spin;
  cube_cell_spin_->setObjectName("cube_cell_spin");
  cube_cell_spin_->setDecimals(3);
  cube_cell_spin_->setRange(0.001, 50.0);
  cube_cell_spin_->setSingleStep(0.01);   // a centimetre: usable at 0.01 m
  cube_cell_spin_->setValue(0.1);
  cube_cell_spin_->setSuffix(" m");
  cube_cell_spin_->setToolTip(
    "CUBE node spacing, 0.001 - 50 m. Finer is not automatically better: a "
    "2-degree beam in 5 m of water has a footprint of about 0.17 m at nadir, "
    "so cells well below that are resolving the sounding pattern rather than "
    "the seafloor. How big a grid a run may allocate is the separate "
    "max-nodes limit in params....");
  cube_cell_spin->setClampNotice(
    [this](double typed, double applied) {
      status_->setText(
        QString(
          "Cell size %1 m is outside %2 - %3 m - using %4 m.")
        .arg(typed, 0, 'g', 4)
        .arg(cube_cell_spin_->minimum())
        .arg(cube_cell_spin_->maximum())
        .arg(applied));
    });
  cube_order_combo_ = new QComboBox(this);
  cube_order_combo_->setObjectName("cube_order_combo");
  for (const auto & order : iho_preset_names()) {
    cube_order_combo_->addItem(QString::fromStdString(order));
  }
  // "custom" is not a preset — it is where the selection lands when the two
  // thresholds are edited to a pair no named order carries (#45).
  cube_order_combo_->addItem(kCustomIhoOrder);
  cube_order_combo_->setToolTip(
    "IHO order — a PRESET for the vertical-uncertainty budget, not the thing "
    "the run reads: it seeds the two thresholds (fixed m + % of depth) in "
    "params…, and the run uses whatever those are. Editing either one moves "
    "this to custom. S-44 order 1a and 1b share one budget (they differ in "
    "the seafloor-search requirement, which CUBE does not model), so they are "
    "one entry here. NOTE: the budget is compared against a PLACEHOLDER "
    "per-sounding error — angle-aware since mpt#49, but still a stand-in; "
    "see params….");
  cube_run_btn_ = new QPushButton("Run CUBE", this);
  cube_run_btn_->setObjectName("cube_run_btn");
  cube_run_btn_->setEnabled(false);   // until a region has been drawn
  cube_run_btn_->setToolTip(
    "Gather every MBES sounding in the map region and CUBE it "
    "at the chosen cell size");
  cube_tuning_ = default_cube_tuning();
  {
    // The ARA curve path survives sessions (a per-sonar file, not a knob
    // you want to re-browse every start).
    const QSettings settings("UNH-CCOM", "survey_explorer");
    cube_tuning_.ara_curve_path =
      settings.value("ara_curve_path").toString().toStdString();
  }
  // The dropdown follows the tuning, never the other way round: it opens on
  // whichever preset the library's own defaults happen to be (#45).
  cube_order_combo_->setCurrentText(
    QString::fromStdString(
      iho_order_for_limits(cube_tuning_.iho_fixed, cube_tuning_.iho_percent)));
  cube_params_btn_ = new QPushButton("params…", this);
  cube_params_btn_->setToolTip(
    "CUBE algorithm parameters (capture scale, uncertainty budget, median "
    "filter, intervention thresholds, extractor) — applied on the next "
    "Run CUBE");
  cube_selfcal_btn_ = new QPushButton("self-cal BS", this);
  cube_selfcal_btn_->setEnabled(false);   // needs a completed run's beams
  cube_selfcal_btn_->setToolTip(
    "Derive the angular-response curve from THIS box's own beams (TL-removed, "
    "2-degree bins), save it as a curve CSV, adopt it and re-run — flattens "
    "whatever gain behaviour the sonar actually has, by construction");
  cube_points_check_ = new QCheckBox("points", this);
  cube_points_check_->setChecked(true);
  cube_points_check_->setToolTip("Show/hide the point cloud");
  cube_surf_check_ = new QCheckBox("surface", this);
  cube_surf_check_->setChecked(true);
  cube_alpha_spin_ = new QDoubleSpinBox(this);
  cube_alpha_spin_->setRange(0.05, 1.0);
  cube_alpha_spin_->setDecimals(2);
  cube_alpha_spin_->setSingleStep(0.1);
  cube_alpha_spin_->setValue(1.0);
  cube_alpha_spin_->setToolTip("Surface opacity");
  cube_shade_combo_ = new QComboBox(this);
  cube_shade_combo_->setObjectName("cube_shade_combo");
  // One vocabulary with the point selector (#36): the same five entries in the
  // same order. Pass is listed and greyed — a node merges every pass that
  // touched it — rather than dropped, so the two lists read as one.
  populateColorVocabulary(cube_shade_combo_);
  for (int i = 0; i < cube_shade_combo_->count(); ++i) {
    set_channel_available(
      cube_shade_combo_, i,
      surface_channel_unavailable_reason(channel_at(cube_shade_combo_, i)));
  }
  cube_shade_combo_->setToolTip(
    "Surface colouring: depth, CUBE uncertainty, CUBE-settled backscatter, "
    "or the draped sidescan pass (pick one in the drape combo). Greyed "
    "entries name a channel the surface cannot carry — the reason is on the "
    "entry.");
  cube_drape_combo_ = new QComboBox(this);
  cube_drape_combo_->addItem("drape: none");
  cube_drape_combo_->setToolTip(
    "Sidescan pass to drape onto the surface (passes crossing the box; "
    "port + starboard of the same interval drape together). Single pass by "
    "design — blending kills shadows.");
  cube_range_score_combo_ = new QComboBox(this);
  cube_range_score_combo_->addItems({"near wins", "mid-range wins"});
  cube_range_score_combo_->setToolTip(
    "Range half of the drape quality score: near wins = closest samples "
    "outrank (best resolution, favours nadir); mid-range wins = the score "
    "peaks mid-swath, penalising nadir distortion AND the far edge (the "
    "classic mosaicking preference)");
  cube_mesh_combo_ = new QComboBox(this);
  cube_mesh_combo_->addItems({"crisp cells", "stepped cells", "blended"});
  cube_mesh_combo_->setToolTip(
    "Surface rendering: crisp cells = one unblended texel per CUBE node on "
    "a smooth watertight relief (the default); stepped cells = the fully "
    "literal view, each node a flat plateau at its own depth; blended = "
    "conventional Gouraud-interpolated colours.");
  cube_srange_.auto_check = new QCheckBox("auto", this);
  cube_srange_.auto_check->setChecked(true);
  cube_srange_.lo = new QDoubleSpinBox(this);
  cube_srange_.hi = new QDoubleSpinBox(this);
  for (auto * s : {cube_srange_.lo, cube_srange_.hi}) {
    s->setRange(-12000.0, 12000.0);
    s->setDecimals(2);
    s->setSingleStep(0.5);
    s->setEnabled(false);
    s->setKeyboardTracking(false);
  }
  const QString srange_tip =
    "Surface colour range in the ACTIVE shade's units (depth m, "
    "uncertainty m, backscatter dB, sidescan amplitude). Auto shows the "
    "computed range in the spins.";
  cube_srange_.auto_check->setToolTip(srange_tip);
  cube_srange_.lo->setToolTip(srange_tip);
  cube_srange_.hi->setToolTip(srange_tip);
  cube_palette_ = new QComboBox(this);
  for (const auto & name : marine_colormap::palette_names()) {
    cube_palette_->addItem(QString::fromStdString(name));
  }
  if (const auto vi = marine_colormap::palette_index("bronze")) {
    cube_palette_->setCurrentIndex(static_cast<int>(*vi));
  }
  cube_palette_->setToolTip(
    "Surface palette — independent of the point cloud's, so cloud and "
    "surface can contrast");

  auto * row = new QHBoxLayout();
  row->setContentsMargins(2, 0, 2, 0);
  row->addWidget(new QLabel("CUBE:", this));
  row->addWidget(cube_cell_spin_);
  row->addWidget(cube_order_combo_);
  row->addWidget(cube_params_btn_);
  row->addWidget(cube_run_btn_);
  row->addWidget(cube_selfcal_btn_);
  row->addStretch(1);
  row->addWidget(cube_points_check_);
  row->addWidget(cube_surf_check_);
  row->addWidget(cube_alpha_spin_);
  row->addWidget(cube_shade_combo_);
  row->addWidget(cube_srange_.auto_check);
  row->addWidget(cube_srange_.lo);
  row->addWidget(cube_srange_.hi);
  row->addWidget(cube_palette_);
  row->addWidget(cube_drape_combo_);
  row->addWidget(cube_range_score_combo_);
  row->addWidget(cube_mesh_combo_);
  // make_pane builds a QVBoxLayout(header, view); the lab row slots between.
  if (auto * v = qobject_cast<QVBoxLayout *>(cloud_pane->layout())) {
    v->insertLayout(1, row);
  }

  // Picking a preset restores that preset's pair; "custom" is only ever
  // arrived at by editing a threshold, so selecting it changes nothing (#45).
  connect(cube_order_combo_, &QComboBox::currentTextChanged, this,
    [this](const QString & name) {
      const auto limits = iho_preset_limits(name.toStdString());
      if (!limits) {
        return;
      }
      cube_tuning_.iho_fixed = limits->first;
      cube_tuning_.iho_percent = limits->second;
    });

  connect(cube_run_btn_, &QPushButton::clicked, this, [this]() {runCubeLab();});
  connect(cube_selfcal_btn_, &QPushButton::clicked,
    this, [this]() {selfCalibrateBackscatter();});
  connect(cube_params_btn_, &QPushButton::clicked, this, [this]() {
      // Modal CUBE-parameter editor, seeded from the current tuning; the
      // Defaults button restores the library's own values. Nothing re-runs
      // automatically — the next Run CUBE picks the tuning up.
      QDialog dialog(this);
      dialog.setWindowTitle("CUBE parameters");
      auto * form = new QFormLayout(&dialog);
      const auto make_dspin = [&dialog](
        double min, double max, double step, int decimals, double value) {
        auto * s = new QDoubleSpinBox(&dialog);
        s->setRange(min, max);
        s->setSingleStep(step);
        s->setDecimals(decimals);
        s->setValue(value);
        return s;
      };
      const auto make_ispin = [&dialog](int min, int max, int value) {
        auto * s = new QSpinBox(&dialog);
        s->setRange(min, max);
        s->setValue(value);
        return s;
      };
      auto * capture = make_dspin(
        0.001, 2.0, 0.01, 3, cube_tuning_.capture_distance_scale);
      capture->setToolTip(
        "Scale on depth for how far out a sounding is accepted "
        "(hydrography ~0.05; larger for sparse/flat areas)");
      // The uncertainty budget, set directly (#45). The order dropdown in the
      // lab row is a preset that seeds these two; editing either takes the
      // selection to "custom", and the run reads these numbers, not the label.
      const QString budget_tip =
      "Vertical-uncertainty budget: max allowed variance at a depth is "
      "(fixed^2 + (percent*depth)^2) / 1.96^2, and its ratio against a "
      "sounding's own error scales the RADIUS over which that sounding "
      "spreads its influence — looser fills in and smooths, tighter is "
      "crisper and holier. Not a pass/fail gate. Editing either value moves "
      "the order dropdown to \"custom\".";
      auto * iho_fixed = make_dspin(
        0.001, 20.0, 0.05, 3, cube_tuning_.iho_fixed);
      iho_fixed->setSuffix(" m");
      iho_fixed->setToolTip(
        budget_tip + "  Fixed part, metres at 95% confidence "
        "(S-44: 0.15 exclusive … 1.0 order 2).");
      auto * iho_percent = make_dspin(
        0.0, 0.5, 0.001, 4, cube_tuning_.iho_percent);
      iho_percent->setToolTip(
        budget_tip + "  Depth-proportional part, as a FRACTION of depth "
        "(S-44: 0.0075 … 0.023).");
      // The caveat belongs where the numbers are set, not only in a header
      // comment (#45): the budget is being compared against a stand-in.
      auto * iho_note = new QLabel(
        QString::fromUtf8(sounding_uncertainty_caveat()), &dialog);
      iho_note->setWordWrap(true);
      iho_note->setMaximumWidth(420);
      {
        QFont f = iho_note->font();
        f.setItalic(true);
        iho_note->setFont(f);
      }
      auto * median = make_ispin(
        1, 101, static_cast<int>(cube_tuning_.median_length));
      median->setToolTip("Median pre-filter sort queue length");
      auto * quotient = make_dspin(1.0, 255.0, 1.0, 1, cube_tuning_.quotient_limit);
      quotient->setToolTip("Outlier quotient upper allowable limit");
      auto * discount = make_dspin(0.5, 1.0, 0.01, 2, cube_tuning_.discount);
      discount->setToolTip("Discount factor for evolution noise variance");
      auto * offset = make_dspin(0.1, 20.0, 0.5, 1, cube_tuning_.estimate_offset);
      offset->setToolTip(
        "Offset from the current estimate (in std devs) that warrants a "
        "new-hypothesis intervention");
      auto * bayes = make_dspin(
        0.001, 10.0, 0.01, 3, cube_tuning_.bayes_factor_threshold);
      bayes->setToolTip("Bayes factor threshold for an intervention");
      auto * runlen = make_ispin(
        1, 100, static_cast<int>(cube_tuning_.runlength_threshold));
      runlen->setToolTip("Run-length threshold for a drift intervention");
      auto * extractor = new QComboBox(&dialog);
      extractor->addItems({"prior (sample count)", "lhood (spatial context)",
        "posterior (combined)"});
      extractor->setCurrentIndex(std::clamp(cube_tuning_.extractor, 0, 2));
      extractor->setToolTip("Multi-hypothesis disambiguation method");
      auto * ara_path = new QLineEdit(
        QString::fromStdString(cube_tuning_.ara_curve_path), &dialog);
      ara_path->setPlaceholderText("(none — raw intensities)");
      ara_path->setToolTip(
        "Per-sonar angular-response curve CSV (cube#81); corrects the "
        "CUBE-settled backscatter for beam angle (removes the bright-nadir "
        "bias). Tier-2 TL terms come from the CSV header.");
      ara_path->setMinimumWidth(280);
      auto * ara_browse = new QPushButton("…", &dialog);
      connect(ara_browse, &QPushButton::clicked, &dialog, [&dialog, ara_path]() {
        const QString start = ara_path->text().isEmpty() ?
        QDir::homePath() + "/data/logs/analysis" :
        QFileInfo(ara_path->text()).absolutePath();
        const QString f = QFileDialog::getOpenFileName(
            &dialog, "Angular-response curve", start, "CSV (*.csv);;All (*)");
        if (!f.isEmpty()) {ara_path->setText(f);}
        });
      auto * ara_row = new QWidget(&dialog);
      auto * ara_lay = new QHBoxLayout(ara_row);
      ara_lay->setContentsMargins(0, 0, 0, 0);
      ara_lay->addWidget(ara_path, 1);
      ara_lay->addWidget(ara_browse);
      auto * max_nodes = make_dspin(
        0.1, 1000000.0, 10.0, 1,
        static_cast<double>(cube_tuning_.max_nodes) / 1e6);
      max_nodes->setSuffix(" M nodes");
      max_nodes->setToolTip(
        "Largest grid a run may allocate before asking (baseline ~20 B/node "
        "+ per-populated-node CUBE state; 100 M ≈ 2 GB). The run "
        "confirmation can override this per run.");
      form->addRow("Capture distance scale", capture);
      form->addRow("Uncertainty budget, fixed", iho_fixed);
      form->addRow("Uncertainty budget, % of depth", iho_percent);
      form->addRow(iho_note);
      form->addRow("Median filter length", median);
      form->addRow("Outlier quotient limit", quotient);
      form->addRow("Evolution discount", discount);
      form->addRow("Intervention offset (σ)", offset);
      form->addRow("Bayes factor threshold", bayes);
      form->addRow("Run-length threshold", runlen);
      form->addRow("Extractor", extractor);
      form->addRow("ARA curve CSV", ara_row);
      form->addRow("Max grid nodes", max_nodes);
      auto * buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel |
        QDialogButtonBox::RestoreDefaults, &dialog);
      form->addRow(buttons);
      connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
      connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
      connect(
        buttons->button(QDialogButtonBox::RestoreDefaults),
        &QPushButton::clicked, &dialog, [&]() {
          const CubeTuning d = default_cube_tuning();
          capture->setValue(d.capture_distance_scale);
          iho_fixed->setValue(d.iho_fixed);
          iho_percent->setValue(d.iho_percent);
          median->setValue(static_cast<int>(d.median_length));
          quotient->setValue(d.quotient_limit);
          discount->setValue(d.discount);
          offset->setValue(d.estimate_offset);
          bayes->setValue(d.bayes_factor_threshold);
          runlen->setValue(static_cast<int>(d.runlength_threshold));
          extractor->setCurrentIndex(std::clamp(d.extractor, 0, 2));
          max_nodes->setValue(static_cast<double>(d.max_nodes) / 1e6);
          ara_path->clear();
        });
      if (dialog.exec() != QDialog::Accepted) {
        return;
      }
      cube_tuning_.capture_distance_scale =
      static_cast<float>(capture->value());
      cube_tuning_.iho_fixed = static_cast<float>(iho_fixed->value());
      cube_tuning_.iho_percent = static_cast<float>(iho_percent->value());
      // Values that are no named order's pair land on "custom"; values that
      // are one restore that preset's name. Guarded so the combo's own
      // preset-seeding slot cannot fight the edit that just happened.
      {
        const QSignalBlocker block(cube_order_combo_);
        cube_order_combo_->setCurrentText(
          QString::fromStdString(
            iho_order_for_limits(
              cube_tuning_.iho_fixed, cube_tuning_.iho_percent)));
      }
      cube_tuning_.median_length = static_cast<std::uint32_t>(median->value());
      cube_tuning_.quotient_limit = static_cast<float>(quotient->value());
      cube_tuning_.discount = static_cast<float>(discount->value());
      cube_tuning_.estimate_offset = static_cast<float>(offset->value());
      cube_tuning_.bayes_factor_threshold = static_cast<float>(bayes->value());
      cube_tuning_.runlength_threshold =
      static_cast<std::uint32_t>(runlen->value());
      cube_tuning_.extractor = extractor->currentIndex();
      cube_tuning_.max_nodes =
      static_cast<std::uint64_t>(max_nodes->value() * 1e6);
      cube_tuning_.ara_curve_path = ara_path->text().toStdString();
      {
        QSettings settings("UNH-CCOM", "survey_explorer");
        settings.setValue(
          "ara_curve_path", QString::fromStdString(cube_tuning_.ara_curve_path));
      }
      status_->setText("CUBE parameters updated — press Run CUBE to apply.");
    });
  connect(cube_points_check_, &QCheckBox::toggled,
    this, [this](bool on) {cloud_->setPointsVisible(on);});
  connect(cube_surf_check_, &QCheckBox::toggled,
    this, [this](bool on) {cloud_->setSurfaceVisible(on);});
  connect(
    cube_alpha_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, [this](double a) {cloud_->setSurfaceAlpha(static_cast<float>(a));});
  connect(cube_shade_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) {refreshCubeSurface();});
  connect(cube_mesh_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) {refreshCubeSurface();});
  connect(cube_srange_.auto_check, &QCheckBox::toggled, this, [this](bool on) {
      cube_srange_.lo->setEnabled(!on);
      cube_srange_.hi->setEnabled(!on);
      refreshCubeSurface();
    });
  connect(cube_srange_.lo, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, [this](double) {
      if (!cube_srange_.auto_check->isChecked()) {refreshCubeSurface();}
    });
  connect(cube_srange_.hi, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, [this](double) {
      if (!cube_srange_.auto_check->isChecked()) {refreshCubeSurface();}
    });
  connect(cube_palette_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) {refreshCubeSurface();});
  connect(
    cube_range_score_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) {requestDrape();});   // re-march with the new score
  connect(cube_drape_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int idx) {
      if (idx <= 0) {
        cube_drape_ = SidescanDrape{};
        cube_drape_terrain_ = CubeSurface{};
        refreshCubeSurface();
        return;
      }
      requestDrape();
    });
  connect(&drape_watcher_, &QFutureWatcher<DrapeTicket>::finished, this, [this]() {
      DrapeTicket ticket = drape_watcher_.result();
      if (ticket.cancelled) {
        return;   // the window is closing (#44): the widgets below are going away
      }
      if (ticket.generation != drape_gen_) {
        return;   // a newer drape (or a new CUBE run) superseded this one
      }
      cube_drape_ = std::move(ticket.drape);
      cube_drape_terrain_ = std::move(ticket.terrain);
      refreshCubeSurface();
      std::size_t painted = 0;
      for (const auto & a : cube_drape_.amplitude) {
        if (std::isfinite(a)) {
          ++painted;
        }
      }
      status_->setText(QString("Drape: %1 pings, %2 cells painted, in %3 s%4")
      .arg(cube_drape_.pings_used)
      .arg(painted)
      .arg(ticket.elapsed_ms / 1000.0, 0, 'f', 1)
      .arg(ticket.notes.isEmpty() ? "" : "  [" + ticket.notes.join("; ") + "]"));
    });
  connect(&cube_watcher_, &QFutureWatcher<CubeLabTicket>::finished, this, [this]() {
      CubeLabTicket ticket = cube_watcher_.result();
      if (ticket.cancelled) {
        return;   // the window is closing (#44): the widgets below are going away
      }
      if (ticket.generation != cube_gen_) {
        return;   // a newer run superseded this one
      }
      cube_surface_ = std::move(ticket.surface);
      if (!cube_surface_.ok()) {
        cloud_->clearSurface();
        status_->setText(QString("CUBE: %1%2")
        .arg(QString::fromStdString(cube_surface_.note))
        .arg(ticket.notes.isEmpty() ? "" : "  [" + ticket.notes.join("; ") + "]"));
        cube_run_btn_->setEnabled(cube_box_.has_value());
        return;
      }
      // A run ADDS a surface over the soundings already on screen (#36): the
      // operator selected a region, got his multi-pass cloud, and asked for a
      // surface over it — replacing that cloud with the run's own gather was
      // read, rightly, as the CUBE run stealing his selection.
      //
      // The one thing that can stop it is the frame: the surface is a grid in
      // the run's own reference world frame and is placed by the displayed
      // cloud's centroid, so it may only be drawn over soundings in that same
      // frame. When the two loads resolved different references the run falls
      // back to showing its own soundings — a misplaced surface would be a
      // wrong answer, not an inconvenience — and says so.
      const bool keep_cloud = cube_surface_shares_cloud_frame(
        selection_cloud_, !cloud_pass_clouds_.empty(),
        selection_ref_bag_, selection_ref_frame_,
        ticket.ref_bag, ticket.ref_frame);
      QString cloud_note;
      if (!keep_cloud) {
        if (selection_cloud_) {
          cloud_note = QString("  [selection cloud replaced: the run's frame "
            "(%1) is not the cloud's (%2)]")
          .arg(QString::fromStdString(
            ticket.ref_bag.empty() ? std::string("none") : ticket.ref_bag))
          .arg(QString::fromStdString(
            selection_ref_bag_.empty() ? std::string("none") : selection_ref_bag_));
        }
        // The lab owns the cloud pane: plain points in the same frame as the
        // surface, scalar modes + range controls live.
        selection_cloud_ = false;
        ++cloud_gen_;   // any tile-selection load in flight is stale
        cloud_pass_clouds_.clear();
        cloud_legend_->clear();
        cloud_legend_->setVisible(false);
        selection_ref_bag_ = ticket.ref_bag;
        selection_ref_frame_ = ticket.ref_frame;
        cloud_->setPoints(ticket.soundings);
        // The pane now shows the RUN's soundings, in the run's reference
        // frame — which may be another bag's than the one open (#47).
        cloud_frame_ = CloudFrame::Reference;
        cloud_ref_anchor_ =
        earthAnchorAffine(ticket.ref_earth_from_world, ticket.ref_has_geo, 0.0);
        refreshCloudColorChannels();   // one set of points: Pass greys out
      }
      cube_soundings_ = std::move(ticket.soundings);   // self-cal input
      cube_selfcal_btn_->setEnabled(!cube_soundings_.empty());
      // The drape frame follows the CUBE load: remember the reference and
      // re-offer the box's sidescan passes; any previous drape is stale.
      cube_ref_bag_ = ticket.ref_bag;
      cube_ref_has_geo_ = ticket.ref_has_geo;
      cube_ref_anchor_ = ticket.ref_earth_from_world;
      ++drape_gen_;
      cube_drape_ = SidescanDrape{};
      cube_drape_terrain_ = CubeSurface{};
      populateDrapePasses();
      refreshCubeSurface();
      status_->setText(QString("CUBE %1 m: %2 soundings, %3 in %4 s%5%6")
      .arg(cube_surface_.cell_m)
      .arg(cube_surface_.soundings_in)
      .arg(QString::fromStdString(cube_surface_.note))
      .arg(ticket.elapsed_ms / 1000.0, 0, 'f', 1)
      .arg(ticket.notes.isEmpty() ? "" : "  [" + ticket.notes.join("; ") + "]")
      .arg(cloud_note));
      cube_run_btn_->setEnabled(cube_box_.has_value());
    });

  connect(canvas_, &SidescanCanvas::cubeBoxSelected, this,
    [this](double s, double w, double n, double e) {
      cube_box_ = GeoRect{s, w, n, e};
      cube_run_btn_->setEnabled(true);
      constexpr double kMetersPerDegLat = 111320.0;
      const double h_m = (n - s) * kMetersPerDegLat;
      const double w_m = (e - w) * kMetersPerDegLat *
      std::max(0.01, std::cos(0.5 * (s + n) * M_PI / 180.0));
      status_->setText(QString("CUBE box: %1 x %2 m — press Run CUBE.")
      .arg(w_m, 0, 'f', 0).arg(h_m, 0, 'f', 0));
    });
  connect(canvas_, &SidescanCanvas::cubeBoxCleared, this, [this]() {
      cube_box_.reset();
      cube_run_btn_->setEnabled(false);
      status_->setText("CUBE box cleared.");
    });
}

void SidescanViewerWindow::selfCalibrateBackscatter()
{
  if (cube_soundings_.empty()) {
    status_->setText("Self-cal: run CUBE first — no beams held.");
    return;
  }
  // Compose with the same absorption the currently-loaded curve uses (the
  // derive and the apply must share alpha); no curve -> 0 (self-consistent).
  double alpha = 0.0;
  if (!cube_tuning_.ara_curve_path.empty()) {
    try {
      alpha = cube::loadAngularResponseCurveWithHeader(
        cube_tuning_.ara_curve_path).absorption_db_per_m;
    } catch (const std::exception &) {
    }
  }
  const QString path = QFileDialog::getSaveFileName(
    this, "Save box self-calibrated ARA curve",
    QDir::homePath() + "/data/logs/analysis/box_selfcal.csv",
    "CSV (*.csv)");
  if (path.isEmpty()) {
    return;
  }
  const std::string err =
    derive_box_curve(cube_soundings_, alpha, path.toStdString());
  if (!err.empty()) {
    QMessageBox::warning(
      this, "Self-calibrate backscatter", QString::fromStdString(err));
    return;
  }
  cube_tuning_.ara_curve_path = path.toStdString();
  {
    QSettings settings("UNH-CCOM", "survey_explorer");
    settings.setValue("ara_curve_path", path);
  }
  status_->setText(
    QString("Self-cal curve written to %1 — re-running CUBE with it.")
    .arg(path));
  runCubeLab();
}

void SidescanViewerWindow::runCubeLab()
{
  if (!bridge_ || !cube_box_) {
    return;
  }
  std::vector<marine_survey_index::PassRow> rows;
  try {
    rows = bridge_->queryBox(
      cube_box_->south, cube_box_->west, cube_box_->north, cube_box_->east,
      "mbes-bathy");
  } catch (const std::exception & e) {
    status_->setText(QString("CUBE pass query failed: %1").arg(e.what()));
    return;
  }
  std::vector<CloudPassInfo> passes;
  for (const auto & p : coalescePasses(rows)) {
    CloudPassInfo info;
    info.bag_path = p.bag_path;
    info.t_start_ns = p.t_start_ns;
    info.t_end_ns = p.t_end_ns;
    info.label = passLabel(p.t_start_ns, p.bag_path);
    passes.push_back(std::move(info));
  }
  if (passes.empty()) {
    status_->setText("CUBE: no MBES passes intersect the box.");
    return;
  }

  // Load clip: the geographic box about its centre (evaluated per pass in
  // its own world frame; the loader's ENU-alignment assumption).
  constexpr double kMetersPerDegLat = 111320.0;
  GeoClip clip;
  clip.lat = 0.5 * (cube_box_->south + cube_box_->north);
  clip.lon = 0.5 * (cube_box_->west + cube_box_->east);
  clip.alt = 0.0;
  clip.half_north_m =
    0.5 * (cube_box_->north - cube_box_->south) * kMetersPerDegLat;
  clip.half_east_m = 0.5 * (cube_box_->east - cube_box_->west) *
    kMetersPerDegLat * std::max(0.01, std::cos(clip.lat * M_PI / 180.0));

  const double cell_m = cube_cell_spin_->value();
  CubeTuning tuning = cube_tuning_;   // the budget rides here, not the label
  // Pre-flight grid estimate from the box itself (known before any loading):
  // over the operator's max-nodes limit, ask — with the real numbers — and
  // let them run anyway (one-shot override; the limit itself is editable in
  // params…). Never a silent refusal.
  {
    const double est_nx = 2.0 * clip.half_east_m / cell_m + 3.0;
    const double est_ny = 2.0 * clip.half_north_m / cell_m + 3.0;
    const double est_nodes = est_nx * est_ny;
    if (est_nodes > static_cast<double>(tuning.max_nodes)) {
      const double base_gb = est_nodes * 20.0 / 1e9;
      const auto answer = QMessageBox::question(
        this, "Large CUBE grid",
        QString("This box at %1 m cells needs a ~%2 x %3 node grid "
        "(~%4 nodes, roughly %5 GB baseline before per-node CUBE state) — "
        "over the max-nodes limit of %6 set in params….\n\nRun anyway?")
        .arg(cell_m)
        .arg(static_cast<qulonglong>(est_nx))
        .arg(static_cast<qulonglong>(est_ny))
        .arg(static_cast<qulonglong>(est_nodes))
        .arg(base_gb, 0, 'f', 1)
        .arg(static_cast<qulonglong>(tuning.max_nodes)),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
      if (answer != QMessageBox::Yes) {
        status_->setText("CUBE run cancelled (grid over the max-nodes limit).");
        return;
      }
      tuning.max_nodes = static_cast<std::uint64_t>(est_nodes * 2.0) + 1;
    }
  }
  ++cube_gen_;
  const auto gen = cube_gen_;
  cube_run_btn_->setEnabled(false);
  status_->setText(QString("CUBE: loading %1 pass%2 + estimating at %3 m …")
    .arg(passes.size()).arg(passes.size() == 1 ? "" : "es").arg(cell_m));
  const auto cancel = supersede_token(cube_cancel_);
  cube_watcher_.setFuture(
    QtConcurrent::run([passes, clip, cell_m, tuning, gen, cancel]() {
      CubeLabTicket ticket;
      ticket.generation = gen;
      QElapsedTimer timer;
      timer.start();
      try {
        auto outcome = load_cloud_passes(passes, clip, cancel);
        if (outcome.cancelled) {
          ticket.cancelled = true;
          return ticket;   // the window is closing: nothing to estimate for
        }
        ticket.notes = std::move(outcome.notes);
        ticket.ref_bag = outcome.ref_bag;
        ticket.ref_frame = outcome.ref_frame;
        ticket.ref_has_geo = outcome.ref_has_geo;
        ticket.ref_earth_from_world = outcome.ref_earth_from_world;
        std::size_t total = 0;
        for (const auto & pc : outcome.pass_clouds) {
          total += pc.size();
        }
        ticket.soundings.reserve(total);
        for (auto & pc : outcome.pass_clouds) {
          ticket.soundings.insert(ticket.soundings.end(), pc.begin(), pc.end());
        }
        ticket.surface = run_cube(ticket.soundings, cell_m, tuning, cancel);
        if (cancel->load(std::memory_order_relaxed)) {
          ticket.cancelled = true;
          ticket.soundings.clear();
          return ticket;
        }
      } catch (const std::exception & e) {
        ticket.surface = CubeSurface{};
        ticket.surface.note = e.what();
      }
      ticket.elapsed_ms = timer.elapsed();
      return ticket;
    }));
}

void SidescanViewerWindow::populateColorVocabulary(QComboBox * combo)
{
  if (!combo) {
    return;
  }
  for (const auto channel : kColorVocabulary) {
    combo->addItem(
      QString::fromUtf8(color_channel_name(channel)), static_cast<int>(channel));
  }
}

// Which point channels the CURRENT cloud can offer. Called after every load
// into the pane, because pass identity comes and goes with the cloud: a
// multi-pass selection has passes to tell apart, a scrub window or a CUBE
// run's own gather is one undifferentiated set of points.
void SidescanViewerWindow::refreshCloudColorChannels()
{
  if (!cloud_color_combo_) {
    return;
  }
  const bool has_pass_identity = cloud_ && cloud_->passCount() > 0;
  int depth_row = 0;
  bool current_unavailable = false;
  for (int i = 0; i < cloud_color_combo_->count(); ++i) {
    const auto channel = channel_at(cloud_color_combo_, i);
    const char * reason =
      point_channel_unavailable_reason(channel, has_pass_identity);
    set_channel_available(cloud_color_combo_, i, reason);
    if (channel == ColorChannel::Depth) {
      depth_row = i;
    }
    if (reason != nullptr && i == cloud_color_combo_->currentIndex()) {
      current_unavailable = true;
    }
  }
  // Never leave a greyed entry showing as the current choice — the pane would
  // claim a colouring it is not drawing.
  if (current_unavailable) {
    cloud_color_combo_->setCurrentIndex(depth_row);
  }
  applyCloudColorMode();
}

void SidescanViewerWindow::applyCloudColorMode()
{
  if (!cloud_ || !cloud_color_combo_) {
    return;
  }
  switch (channel_at(cloud_color_combo_, cloud_color_combo_->currentIndex())) {
    case ColorChannel::Backscatter:
      cloud_->setColorMode(PointCloudView::ColorMode::Backscatter);
      break;
    case ColorChannel::Pass:
      cloud_->setColorMode(PointCloudView::ColorMode::Pass);
      break;
    default:
      // Uncertainty and Sidescan are greyed for the points, so Depth is the
      // only other reachable entry.
      cloud_->setColorMode(PointCloudView::ColorMode::Depth);
      break;
  }
}

void SidescanViewerWindow::refreshCubeSurface()
{
  if (!cube_surface_.ok()) {
    cloud_->clearSurface();
    return;
  }
  const ColorChannel shade_ch = cube_shade_combo_ ?
    channel_at(cube_shade_combo_, cube_shade_combo_->currentIndex()) :
    ColorChannel::Depth;
  const int style_i = cube_mesh_combo_ ? cube_mesh_combo_->currentIndex() : 0;
  const CubeMeshStyle style =
    (style_i == 1) ? CubeMeshStyle::CrispStepped :
    (style_i == 2) ? CubeMeshStyle::Blended : CubeMeshStyle::CrispSmooth;

  // Sidescan shade (#29): colour each node from the drape — painted cells
  // through the SIDESCAN pane's palette + range (so drape and waterfall read
  // identically), acoustic shadows near-black, ensonified-but-unseen nodes
  // dim grey (the relief stays legible).
  if (shade_ch == ColorChannel::Sidescan) {
    // The drape rides its own extended terrain (surface grown to the
    // swath); fall back to the CUBE surface for pre-terrain drapes.
    const CubeSurface & terrain =
      cube_drape_terrain_.ok() ? cube_drape_terrain_ : cube_surface_;
    const std::size_t n_nodes =
      static_cast<std::size_t>(terrain.nx) *
      static_cast<std::size_t>(terrain.ny);
    if (!cube_drape_.ok() ||
      cube_drape_.amplitude.size() != n_nodes)
    {
      status_->setText(
        "Sidescan shade: pick a pass in the drape combo (and re-run after a "
        "new CUBE).");
      cloud_->clearSurface();
      return;
    }
    const auto ss_lut = marine_colormap::bake_lut(
      marine_colormap::palette(static_cast<std::size_t>(
        std::max(0, sidescan_cmap_->currentIndex()))),
      marine_colormap::TransferParams{}, 256);
    float lo = 0.0f;
    float hi = 1.0f;
    if (cube_srange_.auto_check && !cube_srange_.auto_check->isChecked()) {
      // The CUBE row's own range outranks everything.
      lo = static_cast<float>(cube_srange_.lo->value());
      hi = static_cast<float>(cube_srange_.hi->value());
    } else if (ss_range_.auto_check && !ss_range_.auto_check->isChecked()) {
      lo = static_cast<float>(ss_range_.lo->value());
      hi = static_cast<float>(ss_range_.hi->value());
    } else {
      // Auto: robust percentiles of the painted amplitudes (the stores'
      // contrast convention — outliers must not own the ramp).
      std::vector<double> samples;
      samples.reserve(cube_drape_.amplitude.size());
      for (const auto a : cube_drape_.amplitude) {
        if (std::isfinite(a)) {
          samples.push_back(a);
        }
      }
      const auto [rlo, rhi] = robust_range(samples);
      lo = static_cast<float>(rlo);
      hi = static_cast<float>(rhi);
      if (!(hi > lo)) {
        lo = 0.0f;
        hi = 1.0f;
      }
    }
    const float span = (hi > lo) ? (hi - lo) : 1.0f;
    // Colour per node; interpolated terrain that the pass never touched is
    // a HOLE (invented bathymetry must never render as relief), while
    // measured-but-unseen nodes stay dim grey. isfinite(uncertainty) is
    // the measured flag (extend_surface_for_drape leaves it NaN on fills).
    CubeSurface render = terrain;
    std::vector<float> node_rgb(n_nodes * 3, 0.25f);   // unseen = dim grey
    for (std::size_t i = 0; i < n_nodes; ++i) {
      const float a = cube_drape_.amplitude[i];
      const bool measured = std::isfinite(terrain.uncertainty[i]);
      if (std::isfinite(a)) {
        const float t = std::clamp((a - lo) / span, 0.0f, 1.0f);
        const auto & c = ss_lut[static_cast<std::size_t>(
              t * static_cast<float>(ss_lut.size() - 1) + 0.5f)];
        node_rgb[i * 3] = c.r / 255.0f;
        node_rgb[i * 3 + 1] = c.g / 255.0f;
        node_rgb[i * 3 + 2] = c.b / 255.0f;
      } else if (cube_drape_.shadow[i]) {
        node_rgb[i * 3] = 0.05f;   // acoustic shadow: near-black
        node_rgb[i * 3 + 1] = 0.05f;
        node_rgb[i * 3 + 2] = 0.05f;
      } else if (!measured) {
        render.depth[i] = std::nanf("");   // no data, no invented relief
      }
    }
    auto mesh = build_cube_mesh_colored(render, node_rgb, style);
    cloud_->setSurface(
      std::move(mesh.positions), std::move(mesh.colors),
      std::move(mesh.indices));
    if (cube_srange_.auto_check && cube_srange_.auto_check->isChecked()) {
      cube_srange_.lo->blockSignals(true);
      cube_srange_.hi->blockSignals(true);
      cube_srange_.lo->setValue(lo);
      cube_srange_.hi->setValue(hi);
      cube_srange_.lo->blockSignals(false);
      cube_srange_.hi->blockSignals(false);
    }
    return;
  }

  const CubeShade shade =
    (shade_ch == ColorChannel::Uncertainty) ? CubeShade::Uncertainty :
    (shade_ch == ColorChannel::Backscatter) ? CubeShade::Intensity :
    CubeShade::Depth;
  const std::size_t n_pal = marine_colormap::palette_count();
  const auto pal_i = (n_pal > 0) ?
    static_cast<std::size_t>(std::clamp(
      cube_palette_ ? cube_palette_->currentIndex() : 0, 0,
      static_cast<int>(n_pal - 1))) : 0;
  const auto lut = marine_colormap::bake_lut(
    marine_colormap::palette(pal_i), marine_colormap::TransferParams{}, 256);
  std::optional<std::pair<float, float>> range;
  if (cube_srange_.auto_check && !cube_srange_.auto_check->isChecked()) {
    range = std::pair<float, float>(
      static_cast<float>(cube_srange_.lo->value()),
      static_cast<float>(cube_srange_.hi->value()));
  }
  auto mesh = build_cube_mesh(cube_surface_, shade, lut, style, range);
  if (cube_srange_.auto_check && cube_srange_.auto_check->isChecked()) {
    cube_srange_.lo->blockSignals(true);
    cube_srange_.hi->blockSignals(true);
    cube_srange_.lo->setValue(mesh.scalar_lo);
    cube_srange_.hi->setValue(mesh.scalar_hi);
    cube_srange_.lo->blockSignals(false);
    cube_srange_.hi->blockSignals(false);
  }
  cloud_->setSurface(
    std::move(mesh.positions), std::move(mesh.colors), std::move(mesh.indices));
}

std::optional<MapGeoAffine> SidescanViewerWindow::cubeSurfaceAnchor() const
{
  if (!cube_ref_has_geo_ || !cube_surface_.ok()) {
    return std::nullopt;
  }
  // Probe world->geo through the reference earth anchor at the surface's
  // mean depth (the vertical offset moves lat/lon by ~nothing but keeps the
  // ECEF conversion honest).
  double z_sum = 0.0;
  std::size_t z_cnt = 0;
  for (const float d : cube_surface_.depth) {
    if (std::isfinite(d)) {
      z_sum += d;
      ++z_cnt;
    }
  }
  const double z0 = (z_cnt > 0) ? z_sum / static_cast<double>(z_cnt) : 0.0;
  return earthAnchorAffine(cube_ref_anchor_, cube_ref_has_geo_, z0);
}

std::optional<MapGeoAffine> SidescanViewerWindow::earthAnchorAffine(
  const geometry_msgs::msg::TransformStamped & earth_from_world,
  bool has_geo, double z0)
{
  if (!has_geo) {
    return std::nullopt;
  }
  // Probe world->geo through the reference earth anchor at height z0 (the
  // vertical offset moves lat/lon by ~nothing but keeps the ECEF conversion
  // honest).
  const auto & t = earth_from_world.transform;
  return probe_map_anchor(
    [&t, z0](double x, double y, double & lat, double & lon, double & alt) {
      double ex = 0.0;
      double ey = 0.0;
      double ez = 0.0;
      rotate_by_quat(
        t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w,
        x, y, z0, ex, ey, ez);
      ecef_to_geodetic(
        ex + t.translation.x, ey + t.translation.y, ez + t.translation.z,
        lat, lon, alt);
      return true;
    });
}

void SidescanViewerWindow::onExportSurfaceData()
{
  if (!cube_surface_.ok()) {
    status_->setText("Export: run CUBE first — no surface yet.");
    return;
  }
  const auto anchor = cubeSurfaceAnchor();
  if (!anchor) {
    QMessageBox::warning(
      this, "Export CUBE surface",
      "No geographic anchor for the surface's frame — cannot georeference "
      "the GeoTIFF (the CUBE load had no earth reference).");
    return;
  }
  const QString path = QFileDialog::getSaveFileName(
    this, "Export CUBE surface (data bands)", "cube_surface.tif",
    "GeoTIFF (*.tif)");
  if (path.isEmpty()) {
    return;
  }
  const std::string err = write_cube_geotiff_float(
    cube_surface_, *anchor, path.toStdString());
  if (!err.empty()) {
    QMessageBox::warning(
      this, "Export CUBE surface", QString::fromStdString(err));
    return;
  }
  status_->setText(QString(
      "Exported %1x%2 nodes (depth/uncertainty/backscatter Float32) to %3")
    .arg(cube_surface_.nx).arg(cube_surface_.ny).arg(path));
}

void SidescanViewerWindow::onExportSurfaceRgba()
{
  if (!cube_surface_.ok()) {
    status_->setText("Export: run CUBE first — no surface yet.");
    return;
  }
  const auto anchor = cubeSurfaceAnchor();
  if (!anchor) {
    QMessageBox::warning(
      this, "Export CUBE surface",
      "No geographic anchor for the surface's frame — cannot georeference "
      "the GeoTIFF (the CUBE load had no earth reference).");
    return;
  }
  // Render the ACTIVE shade to per-node RGBA, mirroring refreshCubeSurface:
  // alpha 0 = hole (unestimated; or, in the Sidescan shade, interpolated
  // terrain the pass never touched).
  const ColorChannel shade_ch = cube_shade_combo_ ?
    channel_at(cube_shade_combo_, cube_shade_combo_->currentIndex()) :
    ColorChannel::Depth;
  const CubeSurface * srf = &cube_surface_;
  const std::size_t n_scalar =
    static_cast<std::size_t>(cube_surface_.nx) *
    static_cast<std::size_t>(cube_surface_.ny);
  std::vector<std::uint8_t> rgba;
  if (shade_ch == ColorChannel::Sidescan) {
    const CubeSurface & terrain =
      cube_drape_terrain_.ok() ? cube_drape_terrain_ : cube_surface_;
    const std::size_t n =
      static_cast<std::size_t>(terrain.nx) *
      static_cast<std::size_t>(terrain.ny);
    if (!cube_drape_.ok() || cube_drape_.amplitude.size() != n) {
      status_->setText("Export: no drape yet — pick a pass first.");
      return;
    }
    srf = &terrain;
    const auto ss_lut = marine_colormap::bake_lut(
      marine_colormap::palette(static_cast<std::size_t>(
        std::max(0, sidescan_cmap_->currentIndex()))),
      marine_colormap::TransferParams{}, 256);
    float lo = static_cast<float>(cube_srange_.lo->value());
    float hi = static_cast<float>(cube_srange_.hi->value());
    if (!(hi > lo)) {
      lo = 0.0f;
      hi = 1.0f;
    }
    const float span = hi - lo;
    rgba.assign(n * 4, 0);
    for (std::size_t i = 0; i < n; ++i) {
      const float a = cube_drape_.amplitude[i];
      const bool measured = std::isfinite(terrain.uncertainty[i]);
      if (std::isfinite(a)) {
        const float u = std::clamp((a - lo) / span, 0.0f, 1.0f);
        const auto & col = ss_lut[static_cast<std::size_t>(
              u * static_cast<float>(ss_lut.size() - 1) + 0.5f)];
        rgba[i * 4] = col.r;
        rgba[i * 4 + 1] = col.g;
        rgba[i * 4 + 2] = col.b;
        rgba[i * 4 + 3] = 255;
      } else if (cube_drape_.shadow[i]) {
        rgba[i * 4] = 13;
        rgba[i * 4 + 1] = 13;
        rgba[i * 4 + 2] = 13;
        rgba[i * 4 + 3] = 255;
      } else if (measured) {
        rgba[i * 4] = 64;
        rgba[i * 4 + 1] = 64;
        rgba[i * 4 + 2] = 64;
        rgba[i * 4 + 3] = 255;
      }
    }
  } else {
    // Scalar shades: the spins hold the ramp in force (auto keeps them
    // synced to the computed range), so read the ramp straight from them.
    const CubeShade shade =
      (shade_ch == ColorChannel::Uncertainty) ? CubeShade::Uncertainty :
      (shade_ch == ColorChannel::Backscatter) ? CubeShade::Intensity :
      CubeShade::Depth;
    const auto & scalar =
      (shade == CubeShade::Depth) ? cube_surface_.depth :
      (shade == CubeShade::Uncertainty) ? cube_surface_.uncertainty :
      cube_surface_.intensity;
    const std::size_t n_pal = marine_colormap::palette_count();
    const auto pal_i = (n_pal > 0) ?
      static_cast<std::size_t>(std::clamp(
        cube_palette_ ? cube_palette_->currentIndex() : 0, 0,
        static_cast<int>(n_pal - 1))) : 0;
    const auto lut = marine_colormap::bake_lut(
      marine_colormap::palette(pal_i), marine_colormap::TransferParams{}, 256);
    float lo = static_cast<float>(cube_srange_.lo->value());
    float hi = static_cast<float>(cube_srange_.hi->value());
    if (!(hi > lo)) {
      lo = 0.0f;
      hi = 1.0f;
    }
    const float span = hi - lo;
    rgba.assign(n_scalar * 4, 0);
    for (std::size_t i = 0; i < n_scalar; ++i) {
      if (!std::isfinite(cube_surface_.depth[i])) {
        continue;   // hole: alpha 0
      }
      const float v = std::isfinite(scalar[i]) ? scalar[i] : lo;
      const float u = std::clamp((v - lo) / span, 0.0f, 1.0f);
      const auto & col = lut[static_cast<std::size_t>(
            u * static_cast<float>(lut.size() - 1) + 0.5f)];
      rgba[i * 4] = col.r;
      rgba[i * 4 + 1] = col.g;
      rgba[i * 4 + 2] = col.b;
      rgba[i * 4 + 3] = 255;
    }
  }
  const QString path = QFileDialog::getSaveFileName(
    this, "Export CUBE surface (coloured)", "cube_surface_rgba.tif",
    "GeoTIFF (*.tif)");
  if (path.isEmpty()) {
    return;
  }
  const std::string err =
    write_cube_geotiff_rgba(*srf, rgba, *anchor, path.toStdString());
  if (!err.empty()) {
    QMessageBox::warning(
      this, "Export CUBE surface", QString::fromStdString(err));
    return;
  }
  status_->setText(QString("Exported %1x%2 coloured nodes to %3")
    .arg(srf->nx).arg(srf->ny).arg(path));
}

void SidescanViewerWindow::populateDrapePasses()
{
  if (!cube_drape_combo_) {
    return;
  }
  cube_drape_combo_->blockSignals(true);
  cube_drape_combo_->clear();
  cube_drape_combo_->addItem("drape: none");
  drape_passes_.clear();
  if (bridge_ && cube_box_) {
    std::vector<marine_survey_index::PassRow> rows;
    try {
      rows = bridge_->queryBox(
        cube_box_->south, cube_box_->west, cube_box_->north, cube_box_->east,
        "sidescan");
    } catch (const std::exception &) {
      rows.clear();
    }
    // Merge port + starboard of the same bag into one interval entry when
    // they overlap (within the pass-coalescing gap): they drape together —
    // no spatial overlap, shadows survive (design decision on #29).
    constexpr std::int64_t kMergeGapNs = 5000000000LL;
    for (const auto & p : coalescePasses(rows)) {
      bool merged = false;
      for (auto & e : drape_passes_) {
        if (e.bag_path == p.bag_path &&
          p.t_start_ns <= e.t1_ns + kMergeGapNs &&
          e.t0_ns <= p.t_end_ns + kMergeGapNs)
        {
          e.t0_ns = std::min(e.t0_ns, p.t_start_ns);
          e.t1_ns = std::max(e.t1_ns, p.t_end_ns);
          merged = true;
          break;
        }
      }
      if (!merged) {
        drape_passes_.push_back({p.bag_path, p.t_start_ns, p.t_end_ns});
      }
    }
    if (drape_passes_.size() > 1) {
      cube_drape_combo_->addItem(
        QString("composite: best pixel (%1 passes)").arg(drape_passes_.size()));
    }
    for (const auto & e : drape_passes_) {
      cube_drape_combo_->addItem(
        QString::fromStdString(passLabel(e.t0_ns, e.bag_path)));
    }
  }
  cube_drape_combo_->setEnabled(cube_drape_combo_->count() > 1);
  cube_drape_combo_->blockSignals(false);
}

void SidescanViewerWindow::requestDrape()
{
  const int idx = cube_drape_combo_ ? cube_drape_combo_->currentIndex() : 0;
  if (idx <= 0 || !cube_surface_.ok()) {
    return;
  }
  // Item layout: [0] none, [1] composite (only when >1 pass), then passes.
  const bool has_composite = drape_passes_.size() > 1;
  std::vector<DrapePassEntry> targets;
  if (has_composite && idx == 1) {
    targets = drape_passes_;   // best-pixel composite over every pass
  } else {
    const int entry_i = idx - (has_composite ? 2 : 1);
    if (entry_i < 0 || static_cast<std::size_t>(entry_i) >= drape_passes_.size()) {
      return;
    }
    targets.push_back(drape_passes_[static_cast<std::size_t>(entry_i)]);
  }
  const CubeSurface surface = cube_surface_;   // worker's own copy
  const std::string cache_dir = cache_dir_;
  const std::string ref_bag = cube_ref_bag_;
  const bool ref_has_geo = cube_ref_has_geo_;
  const geometry_msgs::msg::TransformStamped ref_anchor = cube_ref_anchor_;
  const std::uint64_t max_nodes = cube_tuning_.max_nodes;
  const RangeScoreMode range_mode =
    (cube_range_score_combo_ && cube_range_score_combo_->currentIndex() == 1) ?
    RangeScoreMode::MidRange : RangeScoreMode::Nearest;
  ++drape_gen_;
  const auto gen = drape_gen_;
  status_->setText(targets.size() == 1 ?
    QString("Draping %1 …")
    .arg(QFileInfo(QString::fromStdString(targets.front().bag_path)).fileName()) :
    QString("Draping composite of %1 passes …").arg(targets.size()));
  const auto cancel = supersede_token(drape_cancel_);
  drape_watcher_.setFuture(QtConcurrent::run(
      [targets, surface, cache_dir, ref_bag, ref_has_geo, ref_anchor,
      max_nodes, range_mode, gen, cancel]() {
        DrapeTicket ticket;
        ticket.generation = gen;
        // A non-QException escaping a QtConcurrent task std::terminates on
        // Qt5, and the throw resurfaces on the UI thread out of result() --
        // or out of the destructor's waitForFinished(), i.e. a throw from a
        // destructor. extend_surface_for_drape() sizes a grid bounded by the
        // operator's own "Run anyway" override, so bad_alloc here is the case
        // the dialog invites rather than a pathological one (#42 review).
        try {
          QElapsedTimer timer;
          timer.start();
          std::vector<WindowPing> pings;
          for (const auto & entry : targets) {
            const auto loaded = load_drape_pings(
              entry.bag_path, entry.t0_ns, entry.t1_ns, cache_dir,
              ref_bag, ref_has_geo, ref_anchor, cancel);
            if (loaded.cancelled) {
              ticket.cancelled = true;
              return ticket;   // the window is closing: nothing to march on
            }
            for (const auto & n : loaded.notes) {
              ticket.notes << QString::fromStdString(n);
            }
            if (loaded.ok) {
              pings.insert(pings.end(), loaded.pings.begin(), loaded.pings.end());
            }
          }
          if (!pings.empty()) {
            // The sidescan outreaches the MBES: extend the surface to the
            // swath (holes filled, edges extrapolated) so the drape has
            // terrain to land on beyond the bathymetry.
            std::string grow_note;
            ticket.terrain = extend_surface_for_drape(
              surface, pings, max_nodes, grow_note, cancel);
            if (cancel->load(std::memory_order_relaxed)) {
              ticket.cancelled = true;
              ticket.terrain = CubeSurface{};
              return ticket;
            }
            if (!grow_note.empty()) {
              ticket.notes << QString::fromStdString(grow_note);
            }
            ticket.drape = drape_pass(ticket.terrain, pings, range_mode, cancel);
            if (cancel->load(std::memory_order_relaxed)) {
              ticket.cancelled = true;
              ticket.terrain = CubeSurface{};
              ticket.drape = SidescanDrape{};
              return ticket;
            }
            if (ticket.drape.pings_skipped > 0) {
              ticket.notes << QString("%1 pings unusable (no altitude/side or "
                "off the surface)").arg(ticket.drape.pings_skipped);
            }
          }
          ticket.elapsed_ms = timer.elapsed();
        } catch (const std::exception & e) {
          ticket.drape = SidescanDrape{};
          ticket.terrain = CubeSurface{};
          ticket.notes << QString("drape failed: %1").arg(e.what());
        } catch (...) {
          ticket.drape = SidescanDrape{};
          ticket.terrain = CubeSurface{};
          ticket.notes << QString("drape failed: unknown exception");
        }
        return ticket;
      }));
}

SidescanViewerWindow::SidescanViewerWindow(QWidget * parent)
: QMainWindow(parent)
{
  setWindowTitle("Survey Explorer");

  canvas_ = new SidescanCanvas(this);

  scrub_ = new QSlider(Qt::Horizontal, this);
  scrub_->setRange(0, 1000);
  scrub_->setEnabled(false);

  grid_spin_ = new QDoubleSpinBox(this);
  grid_spin_->setObjectName("grid_spacing_spin");
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
  status_->setObjectName("status");
  // The status line must never dictate the window size: a long load note was
  // resizing the whole window (desk finding). Long text clips instead.
  status_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

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
  waterfall_->setObjectName("sidescan_waterfall");
  waterfall_->set_ground_range(false);   // raw slant range, not slant->ground (palette set below)

  // MBES backscatter waterfall: one row per detection ping, the beam dB fan
  // across-track (beam-index axis, no metric range lines), newest at top.
  mbes_waterfall_ = new marine_sonar_widgets::WaterfallWidget(this);
  mbes_waterfall_->setObjectName("mbes_waterfall");
  // Across-track-projected: keep slant range (we already supply ground/across-track
  // distances, so no further conversion) and draw across-track range lines.
  mbes_waterfall_->set_ground_range(false);
  mbes_waterfall_->set_range_lines(true);

  // Water-column echogram: the down-channel pings as a depth-vs-distance curtain.
  echogram_ = new marine_sonar_widgets::EchogramWidget(this);
  echogram_->setObjectName("echogram");

  // MBES 3D point cloud: orbit view of the window's soundings (colour mode +
  // Z-exaggeration live).
  cloud_ = new PointCloudView(this);
  cloud_->setObjectName("cloud_view");
  setupCloudControls();

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
  // Basemap controls (survey mode): store layer + its own colormap. Hidden
  // until openSurveyIndex discovers the layers.
  basemap_layer_ = new QComboBox(this);
  basemap_layer_->setToolTip("Store layer rendered as the map basemap");
  basemap_layer_->setVisible(false);
  basemap_cmap_ = make_cmap_combo();
  basemap_cmap_->setToolTip("Basemap colormap (percentile-scaled per layer)");
  basemap_cmap_->setVisible(false);

  // Times display in the system local zone by default (#26); this switches
  // the time bar, tooltips, status messages and pass labels to UTC — the
  // zone of bag stamps and survey_index_query output.
  utc_check_ = new QCheckBox("UTC", this);
  utc_check_->setObjectName("utc_check");
  utc_check_->setChecked(false);
  utc_check_->setToolTip(
    "Display times in UTC instead of local time "
    "(bag stamps and survey_index_query output are UTC)");
  // Apply each combo's initial palette to its widget (combos don't fire on init).
  waterfall_->set_color_map(marine_colormap::palette(sidescan_cmap_->currentIndex()));
  mbes_waterfall_->set_color_map(marine_colormap::palette(mbes_cmap_->currentIndex()));
  echogram_->set_color_map(marine_colormap::palette(echo_cmap_->currentIndex()));
  setupRangeControls();

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

  auto * ss_pane = make_pane(
    "Sidescan", waterfall_,
    {ss_range_.auto_check, ss_range_.lo, ss_range_.hi, sidescan_cmap_});
  auto * bs_pane = make_pane(
    "MBES Backscatter", mbes_waterfall_,
    {bs_range_.auto_check, bs_range_.lo, bs_range_.hi, mbes_cmap_});
  auto * wc_pane = make_pane(
    "Water Column", echogram_,
    {wc_range_.auto_check, wc_range_.lo, wc_range_.hi, echo_cmap_});
  // The cloud pane carries a pass legend beside the 3D view (#24): hidden in
  // scrub mode, shown when a tile selection drives the cloud (per-pass colours).
  cloud_legend_ = new QTreeWidget(this);
  cloud_legend_->setObjectName("cloud_legend");
  cloud_legend_->setHeaderLabels({"Pass", "Soundings"});
  cloud_legend_->setRootIsDecorated(false);
  cloud_legend_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
  cloud_legend_->setVisible(false);
  cloud_split_ = new QSplitter(Qt::Horizontal, this);
  cloud_split_->addWidget(cloud_);
  cloud_split_->addWidget(cloud_legend_);
  cloud_split_->setStretchFactor(0, 1);
  clip_contact_check_ = new QCheckBox("clip to contact", this);
  clip_contact_check_->setToolTip(
    "Load only soundings within the margin of the SELECTED contact — several "
    "passes over a tile is millions of points otherwise");
  clip_margin_spin_ = new QDoubleSpinBox(this);
  clip_margin_spin_->setRange(1.0, 500.0);
  clip_margin_spin_->setValue(25.0);
  clip_margin_spin_->setSuffix(" m");
  clip_margin_spin_->setToolTip("Margin around the selected contact");
  auto * cloud_pane = make_pane(
    "MBES 3D", cloud_split_,
    {clip_contact_check_, clip_margin_spin_, cloud_color_combo_,
      cloud_range_.auto_check, cloud_range_.lo, cloud_range_.hi,
      zexag_spin_, point_size_spin_, cloud_palette_});
  setupCubeLab(cloud_pane);   // the CUBE-lab controls row (#27)

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

  // Left-to-right: contacts | map | 2x2 grid, all resizable. The map pane
  // header carries the basemap layer/colormap combos and the contrast
  // controls (survey mode only); the overlay toggles live in the View menu.
  auto * map_pane = make_pane(
    "Map", canvas_,
    {basemap_layer_, basemap_cmap_,
      map_range_.auto_check, map_range_.lo, map_range_.hi});
  outer_split_ = new QSplitter(Qt::Horizontal, this);
  outer_split_->addWidget(contacts_pane);
  outer_split_->addWidget(map_pane);
  outer_split_->addWidget(grid_split_);
  outer_split_->setStretchFactor(0, 0);
  outer_split_->setStretchFactor(1, 3);
  outer_split_->setStretchFactor(2, 4);

  // Status row: the main readout plus right-aligned hover readouts (populated
  // only in survey/geo mode) — the time of the highlighted nav-track fix
  // (#46) and then the cursor's lat/lon. The time sits beside the lat/lon
  // because they answer the same question about the same pointer: where the
  // cursor is, and when the boat was there.
  hover_geo_ = new QLabel(this);
  hover_geo_->setObjectName("hover_geo");
  hover_geo_->setToolTip(
    "Geographic position of the cursor, named by the pane it is over — "
    "empty whenever the cursor is not over a pane that can place it");
  hover_time_ = new QLabel(this);
  hover_time_->setObjectName("hover_time");
  hover_time_->setToolTip(
    "Time of the highlighted nav-track fix — click the map to cue there");
  auto * status_row = new QWidget(this);
  auto * srow = new QHBoxLayout(status_row);
  srow->setContentsMargins(0, 0, 0, 0);
  srow->addWidget(status_, 1);
  srow->addWidget(hover_time_);
  srow->addWidget(hover_geo_);
  srow->addWidget(utc_check_);   // right under the time bar it switches

  // GeoZui-style time bar (replaced phase d's gap-compressed axis at desk
  // verify): zoomable tape + extent scrollbar under the scrub controls, with
  // the selection's pass bars on it; hidden until there is a time extent.
  time_bar_ = new TimeBarWidget(this);
  time_bar_->setObjectName("time_bar");
  time_bar_->setVisible(false);

  auto * central = new QWidget(this);
  auto * col = new QVBoxLayout(central);
  col->addWidget(outer_split_, 1);
  col->addWidget(controls);
  col->addWidget(time_bar_);
  col->addWidget(status_row);
  setCentralWidget(central);

  auto * file_menu = menuBar()->addMenu("&File");
  file_menu->addAction("&Open Bag…", this, &SidescanViewerWindow::onOpenBag);
  file_menu->addAction(
    "Open Survey &Index…", this, &SidescanViewerWindow::onOpenIndex);
  reopen_index_action_ = file_menu->addAction(
    "Reopen &Last Index", this, &SidescanViewerWindow::onReopenLastIndex);
  refreshReopenIndexAction();
  file_menu->addAction("&Fit View", this, [this]() {
      canvas_->resetView();
      cloud_->resetView();
    });
  file_menu->addSeparator();
  file_menu->addAction("&Load Contacts…", this, &SidescanViewerWindow::onLoadContacts);
  file_menu->addAction("&Save Contacts…", this, &SidescanViewerWindow::onSaveContacts);
  file_menu->addAction(
    "Export Contacts as &GeoJSON…", this, &SidescanViewerWindow::onExportGeoJson);
  file_menu->addSeparator();
  file_menu->addAction(
    "Export CUBE Surface (&Data GeoTIFF)…", this,
    &SidescanViewerWindow::onExportSurfaceData);
  file_menu->addAction(
    "Export CUBE Surface (Colo&ured GeoTIFF)…", this,
    &SidescanViewerWindow::onExportSurfaceRgba);
  file_menu->addSeparator();
  file_menu->addAction("E&xit", this, &QWidget::close);

  // View menu (#42): the map overlay toggles, moved off the map pane header
  // where four checkboxes had crowded out the basemap and contrast controls.
  // Read as a vertical list the labels have to stand on their own, so each
  // one names its overlay in full — above all the two grids, which an
  // operator has already mistaken for each other.
  auto * view_menu = menuBar()->addMenu("&View");
  view_menu->setToolTipsVisible(true);
  const auto add_overlay_action =
    [this, view_menu](const QString & text, const QKeySequence & key,
    const QString & tip) {
      QAction * action = view_menu->addAction(text);
      action->setCheckable(true);
      action->setShortcut(key);
      action->setToolTip(tip);
      // The overlays have nothing to draw until an index opens; each action
      // is enabled there, where its checkbox used to become visible.
      action->setEnabled(false);
      return action;
    };
  show_track_action_ = add_overlay_action(
    "Nav &Track", QKeySequence("Ctrl+1"), "Show the campaign nav track");
  show_track_action_->setObjectName("show_track_action");
  show_track_action_->setChecked(true);
  show_grid_action_ = add_overlay_action(
    "Survey &Index Tile Grid", QKeySequence("Ctrl+2"),
    "Show the index-tile grid (selected tiles stay visible)");
  show_grid_action_->setObjectName("show_grid_action");
  show_grid_action_->setChecked(true);
  // Built-in world coastline (#41): orientation at collection zoom, where the
  // store tiles are specks and nothing else says where you are. It fades out
  // with zoom and is gone before survey scale — see coastline_data.hpp. Its
  // state persists (written on toggle) because an operator who switched it
  // off does not want it back at every launch.
  show_coast_action_ = add_overlay_action(
    "World &Coastline", QKeySequence("Ctrl+3"),
    "Show the built-in world coastline (Natural Earth 1:50m, offline).\n"
    "Orientation only: generalised to the kilometre, drawn under every data "
    "layer, and faded out before survey zoom. Never navigate by it.");
  show_coast_action_->setObjectName("show_coast_action");
  {
    const QSettings settings("UNH-CCOM", "survey_explorer");
    const bool on = settings.value("show_coastline", true).toBool();
    show_coast_action_->setChecked(on);
    canvas_->setCoastlineVisible(on);
  }
  // Metric measuring grid (#42): the slate Cartesian lines and their metre
  // labels, a ruler inherited from the target viewer. Distinct from the tile
  // grid above, which draws the cyan survey index tiles — the two were
  // indistinguishable while only one of them could be switched off, and the
  // header's "grid" / "metric grid" pair barely helped.
  // Checked here for the bag case (the target-viewer window, where the grid
  // IS the measuring tool); openSurveyIndex overrides it for index mode,
  // where the remembered state wins and the default is off.
  show_metric_grid_action_ = add_overlay_action(
    "&Measuring Grid (metres)", QKeySequence("Ctrl+4"),
    "Show the metric measuring grid: Cartesian lines at the spacing set by "
    "the Grid box in the bottom row, labelled in metres from the map "
    "origin.\nA ruler for sizing a target, not navigation chrome — over a "
    "whole collection the origin is arbitrary. This is NOT the survey index "
    "tile grid.");
  show_metric_grid_action_->setObjectName("show_metric_grid_action");
  show_metric_grid_action_->setChecked(true);

  connect(this, &SidescanViewerWindow::indexProgress,
    this, &SidescanViewerWindow::onIndexProgress);
  connect(this, &SidescanViewerWindow::sessionOpened,
    this, &SidescanViewerWindow::onSessionOpened);
  connect(this, &SidescanViewerWindow::openFailed,
    this, &SidescanViewerWindow::onOpenFailed);
  // Scrub-driven opens are debounced: rapid time-bar commits while fine-tuning
  // collapse into one openBag (~2 event-loop breaths after the hand settles).
  open_debounce_.setSingleShot(true);
  open_debounce_.setInterval(400);
  connect(&open_debounce_, &QTimer::timeout, this, [this]() {
      if (!debounce_uri_.empty()) {
        const std::string uri = debounce_uri_;
        debounce_uri_.clear();
        openBag(uri, OpenReason::Cue, debounce_t0_ns_, debounce_t1_ns_);
      }
    });
  connect(&render_watcher_, &QFutureWatcher<SidescanRenderResult>::finished,
    this, &SidescanViewerWindow::onRenderFinished);
  connect(&cloud_watcher_, &QFutureWatcher<CloudLoadTicket>::finished,
    this, &SidescanViewerWindow::onCloudPassesLoaded);
  // Legend checkboxes: show/hide individual passes in the cloud. Unchecked
  // passes become empty slots (indices keep their golden-angle colours);
  // the camera is left alone.
  connect(cloud_legend_, &QTreeWidget::itemChanged, this,
    [this](QTreeWidgetItem *, int) {
      if (!selection_cloud_ || cloud_pass_clouds_.empty()) {
        return;
      }
      std::vector<std::vector<MbesSounding>> visible(cloud_pass_clouds_.size());
      for (int i = 0; i < cloud_legend_->topLevelItemCount() &&
      i < static_cast<int>(cloud_pass_clouds_.size()); ++i)
      {
        if (cloud_legend_->topLevelItem(i)->checkState(0) == Qt::Checked) {
          visible[static_cast<std::size_t>(i)] =
          cloud_pass_clouds_[static_cast<std::size_t>(i)];
        }
      }
      cloud_->setMultiPassPoints(visible);
    });
  // LOD basemap (#26, camp ADR-0013 shape): the loader owns discovery,
  // level selection and demand loads; the canvas's settled-view signal
  // drives it, and every resident-set change re-pushes the paint list.
  basemap_lod_ = new BasemapLod(this);
  connect(basemap_lod_, &BasemapLod::opened, this, [this]() {
      const auto ext = basemap_lod_->dataExtent();
      if (ext) {
        // The basemap bounds are the best fit box: the index extent can be
        // blown out by outlier tiles (junk-GPS passes index far from the
        // survey). Refit unless the operator already took the view over;
        // with an empty index this also establishes the geo origin.
        if (!canvas_->hasGeoOrigin()) {
          canvas_->setGeoOrigin(
            0.5 * (ext->south + ext->north), 0.5 * (ext->west + ext->east));
        }
        if (!canvas_->viewAdjustedByUser()) {
          canvas_->fitGeo(ext->south, ext->west, ext->north, ext->east);
        }
      }
      // Show the sampled auto range in the (disabled) spins, so switching to
      // manual starts from the live scale instead of a stale default.
      if (map_range_.auto_check->isChecked()) {
        map_range_.lo->blockSignals(true);
        map_range_.hi->blockSignals(true);
        map_range_.lo->setValue(basemap_lod_->rangeLo());
        map_range_.hi->setValue(basemap_lod_->rangeHi());
        map_range_.lo->blockSignals(false);
        map_range_.hi->blockSignals(false);
      }
      pushBasemapView();
    });
  connect(basemap_lod_, &BasemapLod::tilesChanged, this, [this]() {
      const auto tiles = basemap_lod_->renderTiles();
      const auto n_tiles = tiles.size();
      canvas_->setStoreTiles(std::move(tiles));
      const int li = basemap_layer_->currentIndex();
      const QString label =
      (li >= 0 && li < static_cast<int>(basemap_layers_.size())) ?
      basemap_layers_[static_cast<std::size_t>(li)].first : QString();
      status_->setText(QString("Basemap %1: %2 tile%3%4")
      .arg(label).arg(n_tiles).arg(n_tiles == 1 ? "" : "s")
      .arg(basemap_lod_->note()));
    });
  connect(canvas_, &SidescanCanvas::viewChanged,
    this, &SidescanViewerWindow::pushBasemapView);
  connect(basemap_layer_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) {requestBasemapLoad();});
  connect(basemap_cmap_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) {requestBasemapLoad();});
  connect(show_track_action_, &QAction::toggled,
    this, [this](bool on) {canvas_->setNavTrackVisible(on);});
  connect(show_grid_action_, &QAction::toggled,
    this, [this](bool on) {canvas_->setIndexTilesVisible(on);});
  connect(show_coast_action_, &QAction::toggled, this, [this](bool on) {
      canvas_->setCoastlineVisible(on);
      QSettings settings("UNH-CCOM", "survey_explorer");
      settings.setValue("show_coastline", on);
    });
  connect(show_metric_grid_action_, &QAction::toggled, this, [this](bool on) {
      canvas_->setMetricGridVisible(on);
      // The spacing spinbox belongs to this grid; grey it out while there is
      // no grid for it to space.
      grid_spin_->setEnabled(on);
      QSettings settings("UNH-CCOM", "survey_explorer");
      settings.setValue("show_metric_grid", on);
    });
  connect(utc_check_, &QCheckBox::toggled, this, [this](bool on) {
      time_bar_->setDisplayUtc(on);
      refreshPassLabels();
      refreshFixHighlightReadout();   // the hovered fix reads in the new zone too
    });

  // Clip changes re-run the selection load (cheap: the query is local, the
  // read is the same windowed machinery).
  const auto reload_selection = [this]() {
      if (selection_cloud_) {
        onTileSelectionChanged();
      }
    };
  connect(clip_contact_check_, &QCheckBox::toggled, this, reload_selection);
  connect(clip_margin_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
    this, reload_selection);
  connect(contact_list_, &QListWidget::currentRowChanged, this, [this](int) {
      if (selection_cloud_ && clip_contact_check_->isChecked()) {
        onTileSelectionChanged();
      }
    });
  connect(canvas_, &SidescanCanvas::tileSelectionChanged,
    this, &SidescanViewerWindow::onTileSelectionChanged);
  connect(canvas_, &SidescanCanvas::hoverGeo,
    this, &SidescanViewerWindow::onMapHoverGeo);
  connect(canvas_, &SidescanCanvas::hoverInterrupted,
    this, &SidescanViewerWindow::clearFixHighlight);
  connect(canvas_, &SidescanCanvas::plainClicked,
    this, &SidescanViewerWindow::onMapPlainClicked);
  connect(time_bar_, &TimeBarWidget::passActivated,
    this, &SidescanViewerWindow::onTimelinePassActivated);
  connect(time_bar_, &TimeBarWidget::timeSelected,
    this, &SidescanViewerWindow::onTimeSelected);
  connect(time_bar_, &TimeBarWidget::centerTimeChanged,
    this, &SidescanViewerWindow::onCenterTimeChanged);
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
  registerMapContextMenuEntries();   // the map menu's window-side entries (#42)
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
  // Right-click a contact to copy its lat/lon (or the whole row) to the clipboard.
  contact_list_->setContextMenuPolicy(Qt::CustomContextMenu);
  connect(contact_list_, &QListWidget::customContextMenuRequested, this,
    [this](const QPoint & pos) {
      QListWidgetItem * item = contact_list_->itemAt(pos);
      if (item == nullptr) {return;}
      const int row = contact_list_->row(item);
      if (row < 0 || row >= static_cast<int>(contact_store_.contacts().size())) {return;}
      const auto & g = contact_store_.contacts()[row].geo_pose.position;
      const bool has_geo = std::isfinite(g.latitude) && std::isfinite(g.longitude);
      QMenu menu(this);
      QAction * copy_ll = menu.addAction("Copy lat, lon");
      copy_ll->setEnabled(has_geo);
      QAction * copy_row = menu.addAction("Copy row");
      QAction * remove = menu.addAction("Delete contact");
      QAction * chosen = menu.exec(contact_list_->viewport()->mapToGlobal(pos));
      if (chosen == copy_ll && has_geo) {
        // The one number format, shared with the status readout and the map
        // menu's Copy Position (#42) — same six decimals, one code path.
        QApplication::clipboard()->setText(
          QString::fromStdString(
            format_geo_coords(GeoPoint{g.latitude, g.longitude})));
      } else if (chosen == copy_row) {
        QApplication::clipboard()->setText(item->text());
      } else if (chosen == remove) {
        // The store's API is add-only: rebuild it without the removed row.
        marine_contacts::ContactStore fresh;
        const auto & all = contact_store_.contacts();
        for (int i = 0; i < static_cast<int>(all.size()); ++i) {
          if (i != row) {
            fresh.add(all[static_cast<std::size_t>(i)]);
          }
        }
        contact_store_ = std::move(fresh);
        refreshContacts();
        if (selection_cloud_ && clip_contact_check_->isChecked()) {
          onTileSelectionChanged();   // the clipped cloud may have lost its contact
        }
      }
    });
  connect(cloud_color_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
    this, [this](int) {applyCloudColorMode();});
  refreshCloudColorChannels();   // the startup cloud is empty: Pass has nothing to tell apart
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

  // Cross-pane linked cursor + middle-click-to-seek. Every pane reports a hovered /
  // clicked world point (the echogram works in along-track fraction); the window
  // broadcasts the cursor to all panes and seeks the scrub on a click.
  connect(canvas_, &SidescanCanvas::hoverWorld, this, &SidescanViewerWindow::onCursorHover);
  connect(canvas_, &SidescanCanvas::seekWorld, this, &SidescanViewerWindow::onCursorSeek);
  connect(cloud_, &PointCloudView::hoverWorld, this, &SidescanViewerWindow::onCursorHover);
  connect(cloud_, &PointCloudView::seekWorld, this, &SidescanViewerWindow::onCursorSeek);
  connectHoverReadout();   // the same hovers again, as the geographic readout (#47)
  const auto wf_hover = [this](QPointF p, bool v) {onCursorHover(p.x(), p.y(), v);};
  const auto wf_seek = [this](QPointF p) {onCursorSeek(p.x(), p.y());};
  connect(waterfall_, &marine_sonar_widgets::WaterfallWidget::hoverMap, this, wf_hover);
  connect(waterfall_, &marine_sonar_widgets::WaterfallWidget::seekRequested, this, wf_seek);
  connect(mbes_waterfall_, &marine_sonar_widgets::WaterfallWidget::hoverMap, this, wf_hover);
  connect(
    mbes_waterfall_, &marine_sonar_widgets::WaterfallWidget::seekRequested, this, wf_seek);
  connect(
    echogram_, &marine_sonar_widgets::EchogramWidget::hoverAlongTrack,
    this, &SidescanViewerWindow::onEchogramHover);
  connect(
    echogram_, &marine_sonar_widgets::EchogramWidget::seekAlongTrack,
    this, &SidescanViewerWindow::onEchogramSeek);

  canvas_->setGridSpacing(grid_spin_->value());
  cache_dir_ = defaultCacheDir();   // --cache-dir overrides via setCacheDir
  resize(1100, 760);

  // Watch app-wide key presses so scrub keys work regardless of which pane has
  // focus (handled in eventFilter; editing widgets keep their own key behaviour).
  qApp->installEventFilter(this);

  // Restore the operator's last window geometry + the resizable-pane splitter sizes.
  // No-op on first run. (Settings key renamed with the app, #24 — the one-time
  // loss of the pre-rename layout is accepted.)
  QSettings settings("UNH-CCOM", "survey_explorer");
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
  restore_split(cloud_split_, "split_cloud");
}

SidescanViewerWindow::~SidescanViewerWindow()
{
  // Don't let a worker outlive the widgets it would signal: wait for any in-flight
  // index/render to finish before the members tear down (~QObject then discards any
  // already-queued indexProgress events targeted at this window). Cancelling the
  // live scan first turns a minutes-long wait into milliseconds.
  // closeEvent already cancelled both tokens on the normal path (#44); this is
  // the backstop for a window destroyed without ever being closed.
  cancelWorkers();
  if (index_watcher_.isRunning()) {index_watcher_.waitForFinished();}
  for (auto & f : superseded_index_futures_) {
    f.waitForFinished();   // orphaned indexers also captured `this`
  }
  if (render_watcher_.isRunning()) {render_watcher_.waitForFinished();}
  if (cloud_watcher_.isRunning()) {cloud_watcher_.waitForFinished();}
  if (cube_watcher_.isRunning()) {cube_watcher_.waitForFinished();}
  if (drape_watcher_.isRunning()) {drape_watcher_.waitForFinished();}
  // basemap_lod_ is a child QObject: its destructor (which cancels + waits
  // its own worker) runs during QObject teardown after this body.
}

void SidescanViewerWindow::cancelWorkers()
{
  // One token per meaning: `scan_cancel_` is the bag index scan's (it doubles
  // as the supersede signal), `worker_cancel_` is the render/cloud/CUBE/drape
  // teardown token. Both are only ever set here, never cleared.
  if (scan_cancel_) {scan_cancel_->store(true);}
  worker_cancel_->store(true);
  // The per-job supersede tokens are what the cloud / CUBE / drape workers
  // actually hold, so teardown has to set them too — worker_cancel_ alone
  // would leave an in-flight job reading its bag to the end.
  if (cloud_cancel_) {cloud_cancel_->store(true);}
  if (cube_cancel_) {cube_cancel_->store(true);}
  if (drape_cancel_) {drape_cancel_->store(true);}
  // basemap_lod_ carries its own cancel_, set by its destructor during the
  // QObject teardown that follows ~SidescanViewerWindow's body.
}

void SidescanViewerWindow::closeEvent(QCloseEvent * event)
{
  // Tell the background workers to stop FIRST (#44): the destructor waits on
  // them, and by the time it runs the window is already off the screen — the
  // operator would be staring at a vanished application while a CUBE run or a
  // multi-bag cloud load finished work nobody wants. Cancelling here gives
  // them the whole teardown to notice.
  cancelWorkers();

  // Persist the window geometry + the resizable-pane splitter sizes so the
  // operator's arrangement survives a restart.
  QSettings settings("UNH-CCOM", "survey_explorer");
  settings.setValue("geometry", saveGeometry());
  if (outer_split_) {settings.setValue("split_outer", outer_split_->saveState());}
  if (grid_split_) {settings.setValue("split_grid", grid_split_->saveState());}
  if (grid_top_split_) {settings.setValue("split_grid_top", grid_top_split_->saveState());}
  if (grid_bot_split_) {settings.setValue("split_grid_bot", grid_bot_split_->saveState());}
  if (cloud_split_) {settings.setValue("split_cloud", cloud_split_->saveState());}
  QMainWindow::closeEvent(event);
}

void SidescanViewerWindow::registerMapContextMenuEntries()
{
  // Copy Position (#42): the operator's reason for it is handing a place to
  // someone else — "generate a surface around here" — so the text has to be
  // pasteable and has to name the same spot he was looking at.
  //
  // Registered by the WINDOW, not the canvas, on the split #42 established:
  // the canvas owns the geometry (and supplies it through contextMenuGeo),
  // while the clipboard and the status row are application state the canvas
  // has no business reaching into.
  canvas_->addContextMenuEntry(
    SidescanCanvas::ContextMenuEntry{
      tr("Copy Position"),
      [this]() {copyMapContextMenuPosition();},
      // Greyed, never hidden and never a zero: with no survey index and no
      // placeable bag the clicked pixel has no position, which is the same
      // condition under which the readout shows nothing (#47).
      [this]() {return canvas_->contextMenuGeo().has_value();}});
}

void SidescanViewerWindow::copyMapContextMenuPosition()
{
  // The position the MENU was opened at, not the cursor's: by now the pointer
  // has travelled down the menu to this entry. The canvas captured it in the
  // context-menu event and has held it since.
  const auto & geo = canvas_->contextMenuGeo();
  if (!geo) {
    // Unreachable through the menu (the entry is greyed out without one), but
    // an action that is asked to copy nothing must copy nothing, not "0, 0".
    return;
  }
  const QString text = QString::fromStdString(format_geo_coords(*geo));
  QApplication::clipboard()->setText(text);
  // Copying is silent otherwise — nothing on screen changes — so the status
  // row is the only thing that can tell him it happened, and it shows what
  // landed on the clipboard so he can check it without pasting.
  status_->setText(tr("Copied %1 to the clipboard.").arg(text));
}

void SidescanViewerWindow::connectHoverReadout()
{
  // Separate from the linked cursor on purpose: the cursor is a map-frame
  // point broadcast to every pane, while the readout is ONE pane's position
  // converted in its own frame and named by the pane that produced it. The
  // map feeds it through hoverGeo (it has a geographic frame of its own) and
  // the echogram through onEchogramHover (its frame is along-track distance).
  connect(
    cloud_, &PointCloudView::hoverWorld, this,
    [this](double x, double y, bool v) {onPaneHoverWorld(HoverPane::Cloud, x, y, v);});
  connect(
    waterfall_, &marine_sonar_widgets::WaterfallWidget::hoverMap, this,
    [this](QPointF p, bool v) {
      onPaneHoverWorld(HoverPane::Sidescan, p.x(), p.y(), v);
    });
  connect(
    mbes_waterfall_, &marine_sonar_widgets::WaterfallWidget::hoverMap, this,
    [this](QPointF p, bool v) {
      onPaneHoverWorld(HoverPane::Mbes, p.x(), p.y(), v);
    });
}

std::optional<HoverPane> SidescanViewerWindow::paneOf(const QObject * obj) const
{
  if (obj == canvas_) {return HoverPane::Map;}
  if (obj == cloud_) {return HoverPane::Cloud;}
  if (obj == waterfall_) {return HoverPane::Sidescan;}
  if (obj == mbes_waterfall_) {return HoverPane::Mbes;}
  if (obj == echogram_) {return HoverPane::Echogram;}
  return std::nullopt;
}

bool SidescanViewerWindow::eventFilter(QObject * obj, QEvent * event)
{
  // The cursor left a pane: its position is no longer live, so it goes (#47).
  // Handled here, on the application filter this window already installs,
  // rather than in each pane: two of the five widgets come from
  // marine_sonar_widgets, which emits no leave — and the alternative, a
  // timer deciding a position had gone stale, would be a guess where Qt has
  // the fact. Never consumed: leaving is the widgets' event too.
  if (event->type() == QEvent::Leave) {
    if (const auto pane = paneOf(obj)) {
      onPaneLeave(*pane);
    }
  }
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
  if (!dir.isEmpty()) {openBag(dir.toStdString(), OpenReason::Explicit);}
}

void SidescanViewerWindow::onOpenIndex()
{
  // Start where the remembered index lives — reloading the usual campaign
  // is one Enter away even without the quick-reload entry.
  const QSettings settings("UNH-CCOM", "survey_explorer");
  const QString last = settings.value("last_index").toString();
  const QString start_dir =
    last.isEmpty() ? QString() : QFileInfo(last).absolutePath();
  const QString path = QFileDialog::getOpenFileName(
    this, "Open survey index", start_dir,
    "Survey index (*.db);;All files (*)");
  if (path.isEmpty()) {
    return;
  }
  openIndexWithDefaults(path.toStdString());
}

void SidescanViewerWindow::openStartupIndex()
{
  // Precedence (#40): the index the operator last had open, then the
  // conventional world collection. An explicit --index never reaches here —
  // main opens it directly. Absence is silent at every step: the window
  // starts empty rather than raising a dialog about a path nobody named.
  const QSettings settings("UNH-CCOM", "survey_explorer");
  const QString last = settings.value("last_index").toString();
  if (!last.isEmpty() && QFileInfo::exists(last)) {
    openIndexWithDefaults(last.toStdString());
    return;
  }
  const auto world = worldIndexPath(defaultWorldRoot());
  std::error_code ec;
  if (!world.empty() && std::filesystem::is_regular_file(world, ec)) {
    openIndexWithDefaults(world.string());
  }
}

void SidescanViewerWindow::onReopenLastIndex()
{
  const QSettings settings("UNH-CCOM", "survey_explorer");
  const QString last = settings.value("last_index").toString();
  if (last.isEmpty()) {
    return;   // action is disabled without a remembered index; belt-and-braces
  }
  openIndexWithDefaults(last.toStdString());
}

void SidescanViewerWindow::openIndexWithDefaults(const std::string & index_path)
{
  // Same stores default as the --stores CLI option: the authoritative depth
  // product beside the index (discovery finds the rest). #40.
  const std::string stores_dir =
    defaultStoresDir(std::filesystem::path(index_path).parent_path()).string();
  try {
    openSurveyIndex(index_path, stores_dir);
  } catch (const std::exception & e) {
    QMessageBox::critical(
      this, "Open survey index failed", QString::fromUtf8(e.what()));
  }
}

void SidescanViewerWindow::refreshReopenIndexAction()
{
  if (!reopen_index_action_) {
    return;
  }
  const QSettings settings("UNH-CCOM", "survey_explorer");
  const QString last = settings.value("last_index").toString();
  const bool usable = !last.isEmpty() && QFileInfo::exists(last);
  reopen_index_action_->setEnabled(usable);
  reopen_index_action_->setText(
    usable ?
    QString("Reopen &Last Index (%1)").arg(QFileInfo(last).fileName()) :
    QString("Reopen &Last Index"));
  if (usable) {
    reopen_index_action_->setToolTip(last);
  }
}

void SidescanViewerWindow::scheduleOpen(
  const std::string & bag_uri, int64_t t0_ns, int64_t t1_ns)
{
  debounce_uri_ = bag_uri;
  debounce_t0_ns_ = t0_ns;
  debounce_t1_ns_ = t1_ns;
  open_debounce_.start();   // restart on every commit; the last one wins
  // Say what the stall IS. Cueing to an instant outside the open recording
  // reopens and re-indexes a whole bag, and in a revisited area — the case
  // the map-click cue exists for (#46) — most clicks land in another
  // recording and pay it. A wait the operator understands is a different
  // thing from one he does not. (Making it CHEAP is the cursor-to-absolute-
  // time work discussed against #36, not this.)
  status_->setText(
    QString("Cueing %1 — that time is in another recording, which has to be "
      "reopened and indexed …")
    .arg(QFileInfo(QString::fromStdString(bag_uri)).fileName()));
}

void SidescanViewerWindow::openBag(
  const std::string & bag_uri, OpenReason reason,
  int64_t cue_start_ns, int64_t cue_end_ns)
{
  // An explicit open frames the bag the operator asked for; a cued one leaves
  // his map zoom/centre and 3D camera exactly where they were (#46). Both fit
  // points below read this, so the deferred one belongs to the same open.
  open_fits_view_ = reason == OpenReason::Explicit;
  // A direct open (menu, pass click) outranks a pending debounced one.
  open_debounce_.stop();
  debounce_uri_.clear();
  // The session is constructed AND indexed on a worker: the UI thread never
  // touches a multi-GB bag synchronously (#26 responsiveness). A bad bag
  // surfaces via openFailed; the session arrives via sessionOpened.

  // Jump-to-pass cue: remembered here, applied once indexing completes (the
  // time→distance mapping needs the finished index). Reset on every open so a
  // bag opened later via the menu doesn't inherit a stale cue. The contract is
  // both-or-neither (the CLI enforces it; programmatic callers may not): a
  // partial cue would swap-normalize into an unintended [0, bound] window, so
  // treat it as no cue.
  const bool has_cue = cue_start_ns != 0 && cue_end_ns != 0;
  pending_cue_start_ns_ = has_cue ? cue_start_ns : 0;
  pending_cue_end_ns_ = has_cue ? cue_end_ns : 0;

  // A previously-running indexer keeps running on its own captured session; bump the
  // epoch so its queued progress is ignored from here on. Its lambda captures
  // `this`, so remember the superseded future for the destructor to wait out
  // (setFuture alone stops watching but neither cancels nor waits), pruning
  // any that already finished.
  if (scan_cancel_) {
    scan_cancel_->store(true);   // stop the abandoned scan's bag streaming
  }
  scan_cancel_ = std::make_shared<std::atomic<bool>>(false);
  if (index_watcher_.isRunning()) {
    superseded_index_futures_.push_back(index_watcher_.future());
  }
  superseded_index_futures_.erase(
    std::remove_if(
      superseded_index_futures_.begin(), superseded_index_futures_.end(),
      [](const QFuture<void> & f) {return f.isFinished();}),
    superseded_index_futures_.end());
  ++index_epoch_;
  const quint64 epoch = index_epoch_;
  session_.reset();   // no session until the worker's open lands (sessionOpened)
  current_bag_uri_ = bag_uri;   // a same-bag timeline cue can skip the re-open
  ++session_epoch_;   // any render in flight for the previous bag is now stale

  // Reset the views for the new bag; the scrub range grows on each progress tick.
  // The geo anchor of the previous bag no longer applies — until this bag's
  // index resolves one, its layers are hidden on the survey map (geo mode) and
  // draw as before in bag-only mode.
  canvas_->setMapAnchor(std::nullopt);
  canvas_->setTrack({});
  if (open_fits_view_) {
    canvas_->resetView();
    // The 3D camera gets the same treatment: it is operator state too, and it
    // is already frame-independent — the pane recentres every cloud on its own
    // centroid and keeps the orbit and zoom across a scrub, which is exactly
    // what a cued reopen is from the operator's side (a new window of
    // soundings, same viewing angle). Fitting it is only right when he asked
    // for this bag.
    cloud_->resetView();
  }
  scrub_->setEnabled(false);
  scrub_->blockSignals(true);
  scrub_->setRange(0, 1);
  scrub_->setValue(0);
  scrub_->blockSignals(false);
  progress_->setVisible(true);
  loading_ = true;
  status_->setText(QString("Opening %1 …").arg(QString::fromStdString(bag_uri)));

  // Open + index off the UI thread; the progress callback emits a queued signal
  // so the UI grows the range + track as the bag resolves. The session lives in
  // the task until sessionOpened hands it over; index_watcher_ is waited on in
  // the destructor. Bag-index cache (#24): a valid cache replaces the whole-bag
  // metadata scan with a file read; a miss scans as before and saves for next
  // time — unless the scan was cancelled, whose partial index must never be
  // cached.
  const std::string cache_path =
    cache_dir_.empty() ? std::string() : cachePathFor(cache_dir_, bag_uri);
  const auto cancel = scan_cancel_;
  index_watcher_.setFuture(
    QtConcurrent::run([this, epoch, bag_uri, cache_path, cancel]() {
      std::shared_ptr<SidescanBagSession> session;
      try {
        session = std::make_shared<SidescanBagSession>(bag_uri);
      } catch (const std::exception & e) {
        Q_EMIT openFailed(epoch, QString::fromStdString(e.what()));
        return;
      }
      {
        QMutexLocker lock(&pending_open_mutex_);
        pending_open_session_ = session;
        pending_open_epoch_ = epoch;
      }
      Q_EMIT sessionOpened(epoch);
      // A non-QException from a QtConcurrent task std::terminates in Qt5 —
      // never let a corrupt cache or a failing scan out of the worker
      // (review round-2 finding); a failure just ends progress where it is.
      try {
        if (!cache_path.empty()) {
          const auto identity = bagIdentity(bag_uri);
          if (auto cached = loadSessionIndex(cache_path, identity)) {
            const double total = cached->total_distance_m;
            session->adoptIndex(std::move(*cached));
            Q_EMIT indexProgress(epoch, total, true);
            return;
          }
          session->buildIndex(
            [this, epoch](double resolved_m, bool done) {
              Q_EMIT indexProgress(epoch, resolved_m, done);
            }, cancel);
          if (cancel->load(std::memory_order_relaxed)) {
            return;   // superseded: a partial index must not poison the cache
          }
          if (const auto snap = session->snapshot()) {
            saveSessionIndex(cache_path, identity, *snap);   // best-effort
          }
          return;
        }
        session->buildIndex(
          [this, epoch](double resolved_m, bool done) {
            Q_EMIT indexProgress(epoch, resolved_m, done);
          }, cancel);
      } catch (const std::exception &) {
        Q_EMIT indexProgress(epoch, 0.0, true);   // surface as an empty done
      }
    }));
}

void SidescanViewerWindow::onSessionOpened(quint64 epoch)
{
  std::shared_ptr<SidescanBagSession> session;
  {
    QMutexLocker lock(&pending_open_mutex_);
    if (pending_open_epoch_ != epoch) {
      return;   // a newer open already parked its session; wait for its signal
    }
    session = std::move(pending_open_session_);
    pending_open_session_.reset();
  }
  if (epoch != index_epoch_ || !session) {
    return;   // superseded while opening: discard (its scan is cancelled)
  }
  session_ = std::move(session);
  status_->setText(
    QString("Indexing %1 …").arg(QString::fromStdString(current_bag_uri_)));
}

void SidescanViewerWindow::onOpenFailed(quint64 epoch, const QString & message)
{
  if (epoch != index_epoch_) {
    return;   // a stale open's failure is moot
  }
  loading_ = false;
  progress_->setVisible(false);
  current_bag_uri_.clear();
  status_->setText("Open a bag to begin (File → Open Bag).");
  QMessageBox::critical(this, "Open bag failed", message);
}

void SidescanViewerWindow::onIndexProgress(quint64 epoch, double resolved_m, bool done)
{
  if (epoch != index_epoch_ || !session_) {return;}   // stale (a newer bag was opened)

  const bool first = !scrub_->isEnabled();
  const int max_m = std::max(1, static_cast<int>(std::lround(resolved_m)));
  scrub_->setEnabled(true);
  const int cur = scrub_->value();
  scrub_->blockSignals(true);
  scrub_->setRange(0, max_m);
  scrub_->setValue(std::min(cur, max_m));
  scrub_->blockSignals(false);
  updateScrubStep();

  // Grow the boat-track polyline from the snapshot. Decimated for display:
  // trackPoints() is per POSED PING (~600k on a long bag), and the bag track
  // is a dynamic canvas overlay redrawn every repaint — feeding it raw made
  // the whole UI sluggish the moment a bag finished loading (desk finding).
  // ~4k points is indistinguishable at any zoom the map reaches.
  const auto pts = session_->trackPoints();
  const std::size_t stride = std::max<std::size_t>(1, pts.size() / 4000);
  std::vector<QPointF> track;
  track.reserve(pts.size() / stride + 2);
  for (std::size_t i = 0; i < pts.size(); i += stride) {
    track.emplace_back(pts[i].first, pts[i].second);
  }
  if (!pts.empty() && (pts.size() - 1) % stride != 0) {
    track.emplace_back(pts.back().first, pts.back().second);
  }
  canvas_->setTrack(track);
  if (first && open_fits_view_) {
    canvas_->resetView();   // fit once when the first resolved data arrives
    cloud_->resetView();
  }
  requestRender();

  if (done) {
    loading_ = false;
    progress_->setVisible(false);
    // Place this bag on the survey map (#24): probe the session's mapToGeo
    // into an affine anchor. nullopt (no earth reference in the bag) keeps the
    // bag's layers hidden in geo mode rather than placing them by guesswork.
    canvas_->setMapAnchor(probe_map_anchor(
        [this](double x, double y, double & lat, double & lon, double & alt) {
          return session_->mapToGeo(x, y, lat, lon, alt);
        }));
    // Time bar: only BAG-ONLY mode uses the bag's span — with an index the
    // campaign extent is authoritative, and replacing it on a cross-bag cue
    // left the whole extent a few pixels wide at campaign zoom (desk finding).
    if (selection_passes_.empty() && nav_track_points_.empty()) {
      const double t0 = session_->timeAtDistance(0.0);
      const double t1 = session_->timeAtDistance(session_->totalDistance());
      time_bar_->setExtent(
        static_cast<std::int64_t>(t0 * 1e9), static_cast<std::int64_t>(t1 * 1e9));
      time_bar_->setVisible(time_bar_->hasExtent());
    }
    status_->setText(QString(
        "%1 pings (%2 port, %3 stbd, %4 down) • %5 m track • alt: %6 • geo: %7")
      .arg(session_->pingCount())
      .arg(session_->channelCount(SidescanChannel::Port))
      .arg(session_->channelCount(SidescanChannel::Starboard))
      .arg(session_->channelCount(SidescanChannel::Down))
      .arg(session_->totalDistance(), 0, 'f', 1)
      .arg(session_->usedNadirDepth() ? "nadir_depth" : "estimator")
      .arg(session_->hasGeoReference() ? "yes" : "no"));
    // Apply a pending jump-to-pass cue now that the scrub range is final. The
    // scrub head paints the TRAILING window [head − window_len, head], so cue
    // head = min(lo + window_len, hi): a pass shorter than the window lands
    // fully in view (with leading context); a longer one opens on its first
    // window-length.
    if (pending_cue_start_ns_ != 0 || pending_cue_end_ns_ != 0) {
      const auto snapshot = session_->snapshot();
      const auto interval = snapshot ?
        distance_interval(*snapshot, pending_cue_start_ns_, pending_cue_end_ns_) :
        std::nullopt;
      if (interval) {
        const double head = std::min(interval->first + window_len_m_, interval->second);
        // valueChanged re-renders on a move; if head equals the current value
        // the earlier requestRender() in this handler already covers it.
        scrub_->setValue(static_cast<int>(std::lround(head)));
        status_->setText(status_->text() + QString(" • cued to %1–%2 m")
          .arg(interval->first, 0, 'f', 0).arg(interval->second, 0, 'f', 0));
      } else {
        status_->setText(status_->text() + " • cue window matched no posed pings");
      }
      pending_cue_start_ns_ = 0;
      pending_cue_end_ns_ = 0;
    }
  } else {
    status_->setText(QString("Indexing… %1 m resolved").arg(resolved_m, 0, 'f', 0));
  }
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
  std::vector<marine_contacts::MapPoint> pts{
    {map_rect.left(), map_rect.top()},
    {map_rect.right(), map_rect.bottom()}};
  const QString id = QString("T-%1").arg(++contact_counter_, 3, 10, QChar('0'));
  const double head = std::clamp(static_cast<double>(scrub_->value()), 0.0,
    session_ ? session_->totalDistance() : 0.0);
  const double stamp_s = session_ ? session_->timeAtDistance(head) : 0.0;
  auto contact = marine_contacts::make_box_contact(
    pts, id.toStdString(), "sidescan", "bizzy/map", stamp_s);

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

void SidescanViewerWindow::onExportGeoJson()
{
  if (contact_store_.size() == 0) {
    QMessageBox::information(this, "Export GeoJSON", "No contacts to export.");
    return;
  }
  QString path = QFileDialog::getSaveFileName(
    this, "Export contacts as GeoJSON", QString(), "GeoJSON (*.geojson)");
  if (path.isEmpty()) {return;}
  if (!path.endsWith(".geojson", Qt::CaseInsensitive)) {path += ".geojson";}
  const auto r = marine_contacts::export_contacts_geojson(
    contact_store_.contacts(), path.toStdString());
  if (!r.ok) {
    QMessageBox::warning(this, "Export failed", "Could not write " + path);
    return;
  }
  QString msg = QString("Exported %1 contact(s) to GeoJSON.").arg(r.written);
  if (r.skipped > 0) {
    msg += QString(" %1 skipped (no geo reference — open a bag with an earth→map "
      "transform to resolve lat/lon).").arg(r.skipped);
  }
  status_->setText(msg);
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

void SidescanViewerWindow::onCursorHover(double map_x, double map_y, bool valid)
{
  // Broadcast the shared cursor to every pane. The source pane drawing its own
  // cursor too is harmless (confirms the projection).
  if (!valid) {
    canvas_->setCursorWorld(std::nullopt);
    waterfall_->setCursorPoint(std::nullopt);
    mbes_waterfall_->setCursorPoint(std::nullopt);
    cloud_->setCursorWorld(0.0, 0.0, false);
    echogram_->setCursorAlongTrack(std::nullopt);
    return;
  }
  const QPointF p(map_x, map_y);
  canvas_->setCursorWorld(p);
  waterfall_->setCursorPoint(p);
  mbes_waterfall_->setCursorPoint(p);
  cloud_->setCursorWorld(map_x, map_y, true);
  // Echogram: map the world point to an along-track fraction within the window.
  double d = 0.0;
  const double span = last_win_hi_ - last_win_lo_;
  if (session_ && span > 0.0 && session_->nearestTrackDistance(map_x, map_y, d)) {
    const double frac = (d - last_win_lo_) / span;
    echogram_->setCursorAlongTrack(
      (frac >= 0.0 && frac <= 1.0) ? std::optional<double>(frac) : std::nullopt);
  } else {
    echogram_->setCursorAlongTrack(std::nullopt);
  }
}

void SidescanViewerWindow::onCursorSeek(double map_x, double map_y)
{
  double d = 0.0;
  if (session_ && session_->nearestTrackDistance(map_x, map_y, d)) {
    scrub_->setValue(std::clamp(
        static_cast<int>(std::lround(d)), scrub_->minimum(), scrub_->maximum()));
  }
}

void SidescanViewerWindow::onEchogramHover(double frac, bool valid)
{
  // The echogram's own frame is along-track distance: a column is where the
  // boat was, so its position is the track position at that distance. With no
  // open bag, or a distance the track cannot answer for, it is nothing.
  const double span = last_win_hi_ - last_win_lo_;
  double x = 0.0;
  double y = 0.0;
  const bool placed = valid && session_ && span > 0.0 &&
    session_->positionAtDistance(last_win_lo_ + frac * span, x, y);
  onCursorHover(x, y, placed);
  onPaneHoverWorld(HoverPane::Echogram, x, y, placed);
}

void SidescanViewerWindow::onEchogramSeek(double frac)
{
  const double span = last_win_hi_ - last_win_lo_;
  if (!session_ || !(span > 0.0)) {return;}
  const double d = last_win_lo_ + frac * span;
  scrub_->setValue(std::clamp(
      static_cast<int>(std::lround(d)), scrub_->minimum(), scrub_->maximum()));
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
    // lat/lon in decimal degrees when the bag resolved a geo reference, else a marker.
    const double lat = c.geo_pose.position.latitude;
    const double lon = c.geo_pose.position.longitude;
    const QString geo = (std::isfinite(lat) && std::isfinite(lon)) ?
      QString("%1, %2").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6) :
      QStringLiteral("no geo");
    contact_list_->addItem(QString("%1   %2 x %3 m   (%4, %5)   %6")
      .arg(m.id)
      .arg(m.w, 0, 'f', 1).arg(m.h, 0, 'f', 1)
      .arg(p.x, 0, 'f', 1).arg(p.y, 0, 'f', 1)
      .arg(geo));
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
  // Closing (#44): never launch a fresh render into the teardown — the
  // destructor would then wait for a job that started after the cancel.
  if (worker_cancel_->load(std::memory_order_relaxed)) {return;}
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
  // Manual amplitude range from the sidescan pane's controls (#26): the map
  // coverage overlay renders the same data, so it follows the same range.
  std::optional<std::pair<float, float>> manual_range;
  if (ss_range_.auto_check && !ss_range_.auto_check->isChecked()) {
    manual_range = {static_cast<float>(ss_range_.lo->value()),
      static_cast<float>(ss_range_.hi->value())};
  }
  const auto cancel = worker_cancel_;
  render_watcher_.setFuture(QtConcurrent::run(
      [session, head, total, win, max_pings, res, palette, epoch, manual_range,
      cancel]() {
        SidescanRenderResult r;
        try {
          r = render_window(
            session, head, total, win.lo, win.hi, max_pings, res, palette,
            manual_range, cancel);
        } catch (const std::exception &) {
          r.ok = false;   // e.g. the bag became unreadable mid-session
          r.cancelled = cancel->load(std::memory_order_relaxed);
        }
        r.epoch = epoch;
        return r;
      }));
}

void SidescanViewerWindow::onRenderFinished()
{
  rendering_ = false;
  const SidescanRenderResult r = render_watcher_.result();
  if (r.cancelled) {
    // The window is closing (#44): publish nothing, and above all do not
    // relaunch a pending render into widgets that are going away.
    render_pending_ = false;
    return;
  }

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

  last_win_lo_ = r.win_lo;   // for the echogram linked-cursor along-track mapping
  last_win_hi_ = r.win_hi;
  // Keep the time bar's centre cursor on the scrub position (it ignores this
  // mid-interaction, so the operator's hand wins).
  if (session_ && time_bar_->hasExtent()) {
    time_bar_->setCurrentTime(
      static_cast<std::int64_t>(session_->timeAtDistance(r.head_m) * 1e9));
  }
  canvas_->setCoverage(r.image, r.origin_x, r.origin_y, r.res_m);
  // Rebuild the sidescan waterfall for this window: size the scrollback to the
  // window so none of its rows are evicted (the lib widget's default 200-row history
  // is smaller than a dense window), then clear and append oldest-first so the newest
  // ping scrolls to the top (matching the live plugin).
  // GL history caps (#24 desk crash): a huge Max-pings window uploads one
  // texture row per ping; bound what reaches the GPU and keep the NEWEST rows.
  constexpr std::size_t kMaxGlRows = 4000;
  const std::size_t ss_first =
    r.sidescan_rows.size() > kMaxGlRows ? r.sidescan_rows.size() - kMaxGlRows : 0;
  if (!r.sidescan_rows.empty()) {
    waterfall_->set_history(r.sidescan_rows.size() - ss_first);
  }
  waterfall_->clear();
  for (std::size_t i = ss_first; i < r.sidescan_rows.size(); ++i) {
    waterfall_->add_row(r.sidescan_rows[i]);
  }
  // The 3D cloud keeps its own palette (set in the ctor and via cloud_palette_);
  // setPoints recolours with that stored palette, so no per-render setColorMap here.
  // With a tile selection driving the cloud (#24) the scrub render leaves the
  // pane alone — the selection cloud spans bags and would be clobbered.
  if (!selection_cloud_) {
    cloud_->setPoints(r.mbes_soundings);
    cloud_frame_ = CloudFrame::OpenBag;   // the open bag's map-ENU (#47)
    refreshCloudColorChannels();   // one set of points: no pass identity
    cloud_->setBoat(r.boat_x, r.boat_y, r.boat_z, r.boat_heading, r.boat_valid);
  }
  // Size the MBES backscatter scrollback to the window too (same reason as the
  // sidescan pane): the lib's default 200-row history is smaller than a dense
  // detections window, so without this the oldest MBES pings are evicted and the
  // backscatter pane falls out of lockstep with the other panes on the same scrub.
  const std::size_t bs_first = r.mbes_backscatter_rows.size() > kMaxGlRows ?
    r.mbes_backscatter_rows.size() - kMaxGlRows : 0;
  if (!r.mbes_backscatter_rows.empty()) {
    mbes_waterfall_->set_history(r.mbes_backscatter_rows.size() - bs_first);
  }
  mbes_waterfall_->clear();
  for (std::size_t i = bs_first; i < r.mbes_backscatter_rows.size(); ++i) {
    mbes_waterfall_->add_row(r.mbes_backscatter_rows[i]);
  }
  // Echogram has no clear(); sizing history to the window count makes the new
  // pings evict the previous window's, so the curtain shows just this window.
  if (!r.down_images.empty()) {
    echogram_->setHistory(static_cast<int>(r.down_images.size()));
    echogram_->addPings(r.down_images);
  }
  if (r.has_center) {canvas_->setCenter(r.center_x, r.center_y);}
  status_->setText(QString("scrub %1 / %2 m • window [%3, %4] m • %5 pings painted%6")
    .arg(r.head_m, 0, 'f', 1)
    .arg(r.total_m, 0, 'f', 1)
    .arg(r.win_lo, 0, 'f', 1)
    .arg(r.win_hi, 0, 'f', 1)
    .arg(r.npings)
    .arg([&]() {
      QString extra;
      if (cloud_->decimationStride() > 1) {
        extra += QString(" • cloud 1/%1 (render budget)")
        .arg(cloud_->decimationStride());
      }
      if (ss_first > 0 || bs_first > 0) {
        extra += QString(" • waterfalls newest %1 rows").arg(kMaxGlRows);
      }
      return extra;
    }()));

  // A scrub arrived while we were rendering — render once more with the latest.
  if (render_pending_) {
    render_pending_ = false;
    requestRender();
  }
}

// --- survey-explorer mode (#24) ---------------------------------------------

void SidescanViewerWindow::openSurveyIndex(
  const std::string & index_path, const std::string & stores_dir)
{
  bridge_ = std::make_unique<SurveyIndexBridge>(index_path);   // throws on a bad DB
  setWindowTitle(QString("Survey Explorer — %1")
    .arg(QString::fromStdString(index_path)));
  {
    // Remember the index that actually opened (past the throwing ctor) for
    // File -> Reopen Last Index and as the Open dialog's start directory.
    QSettings settings("UNH-CCOM", "survey_explorer");
    settings.setValue(
      "last_index",
      QFileInfo(QString::fromStdString(index_path)).absoluteFilePath());
  }
  refreshReopenIndexAction();

  // The canvas-metre plane needs a geographic origin before layers derive
  // their geometry: the index extent's centre. With an empty index the origin
  // waits for the first basemap load (the geo layer data below is stored
  // regardless and re-derives when the origin lands).
  const auto box = bridge_->extent();
  if (box) {
    canvas_->setGeoOrigin(
      0.5 * (box->south + box->north), 0.5 * (box->west + box->east));
  }

  // Nav track (schema v2, #265): one polyline per bag, segmented at bag_id
  // changes (the accessor orders by bag then time). The rows are kept for
  // the time-bar position arrow (time -> fix) and time -> bag resolution.
  nav_track_points_ = bridge_->navTrack();
  bag_paths_ = bridge_->bagPaths();
  clearFixHighlight();   // the old index's fix is not in this one (#46)
  std::vector<std::vector<std::pair<double, double>>> segments;
  {
    std::int64_t cur_bag = -1;
    for (const auto & p : nav_track_points_) {
      if (p.bag_id != cur_bag) {
        segments.emplace_back();
        cur_bag = p.bag_id;
      }
      segments.back().emplace_back(p.latitude, p.longitude);
    }
  }
  canvas_->setNavTrack(std::move(segments));

  // The time bar is present from startup in index mode, spanning the whole
  // campaign (the nav track's time range); a tile selection only adds pass
  // bars, and an open bag lives inside this extent.
  if (!nav_track_points_.empty()) {
    std::int64_t t0 = nav_track_points_.front().t_ns;
    std::int64_t t1 = t0;
    for (const auto & p : nav_track_points_) {
      t0 = std::min(t0, p.t_ns);
      t1 = std::max(t1, p.t_ns);
    }
    time_bar_->setExtent(t0, t1);
    time_bar_->setCurrentTime(t0);   // arrow starts at the campaign's first fix
    time_bar_->setVisible(true);
  }

  // Selectable index-tile grid; canvas selection indices map into indexed_tiles_.
  indexed_tiles_ = bridge_->indexedTiles();
  std::vector<GeoRect> rects;
  rects.reserve(indexed_tiles_.size());
  for (const auto & t : indexed_tiles_) {
    rects.push_back(GeoRect{t.south, t.west, t.north, t.east});
  }
  canvas_->setIndexTiles(rects);

  if (box) {
    canvas_->fitGeo(box->south, box->west, box->north, box->east);
    status_->setText(
      QString("Survey index: %1 tiles indexed — drag on the map to select a "
        "region and load its passes; click to clear.")
      .arg(indexed_tiles_.size()));
  } else {
    status_->setText("Survey index holds no passes — nothing to explore.");
  }

  // Basemap: discover the store layers next to the index and load the
  // initial one (async; layer/colormap combos re-load on change).
  discoverBasemapLayers(
    std::filesystem::path(index_path).parent_path().string(), stores_dir);
  requestBasemapLoad();
  show_track_action_->setEnabled(true);
  show_grid_action_->setEnabled(true);
  show_coast_action_->setEnabled(true);
  // Index mode: the measuring grid is off unless the operator asked for it
  // back. Applied here rather than in the constructor because this is the
  // moment the window learns which mode it is in.
  {
    const QSettings settings("UNH-CCOM", "survey_explorer");
    const bool on = settings.value("show_metric_grid", false).toBool();
    show_metric_grid_action_->setChecked(on);
    canvas_->setMetricGridVisible(on);
    grid_spin_->setEnabled(on);
  }
  show_metric_grid_action_->setEnabled(true);
  loadCoastlineLayer();
}

void SidescanViewerWindow::loadCoastlineLayer()
{
  if (coastline_loaded_) {
    return;   // the canvas keeps the data across index opens
  }
  coastline_loaded_ = true;   // one attempt per session, warning included
  std::string share;
  try {
    share = ament_index_cpp::get_package_share_directory("marine_perception_tools");
  } catch (const std::exception & e) {
    qWarning() << "No coastline layer: package share directory not found —" << e.what();
    return;
  }
  const std::string path =
    (std::filesystem::path(share) / "data" / "coastline" / "ne_50m_coastline.txt").string();
  // Decimate at ~55 m, half the milli-degree quantisation of the vendored
  // data: it removes the collinear runs that quantisation leaves behind
  // without moving the line anywhere the eye could follow.
  auto coastline = loadCoastline(path, 0.0005);
  if (coastline.empty()) {
    qWarning() << "No coastline layer: nothing usable in"
               << QString::fromStdString(path);
    return;
  }
  canvas_->setCoastline(std::move(coastline));
}

void SidescanViewerWindow::discoverBasemapLayers(
  const std::string & root, const std::string & initial_dir)
{
  basemap_layers_.clear();

  const auto has_tif = [](const std::filesystem::path & dir) {
      std::error_code ec;
      for (const auto & e : std::filesystem::directory_iterator(dir, ec)) {
        if (e.path().extension() == ".tif") {
          return true;
        }
      }
      return false;
    };

  // Preferred layers first (the ones an operator reaches for), then any other
  // tile-holding subdirectory found one or two levels under the stores root.
  // The names are the uma-ADR-0010 D3 taxonomy; see world_layout.hpp (#40).
  for (const auto & rel : preferredLayerPaths()) {
    const auto dir = std::filesystem::path(root) / rel;
    if (has_tif(dir)) {
      basemap_layers_.emplace_back(QString::fromStdString(rel), dir.string());
    }
  }
  // Fallback discovery: any tile-holding directory up to three levels under
  // the root. Three, not two, because the imagery theme nests a store between
  // the theme and its layers (imagery/sidescan/processed) while the depth
  // theme does not (depths/processed). A directory that is not a world
  // collection at all still lights up whatever it has.
  const std::function<void(const std::filesystem::path &, int)> scan =
    [&](const std::filesystem::path & dir, int depth) {
      if (depth > 3) {
        return;
      }
      std::error_code ec;
      for (const auto & e : std::filesystem::directory_iterator(dir, ec)) {
        if (!e.is_directory()) {
          continue;
        }
        // The derived overview sidecar is the same layer at coarser levels,
        // not a layer of its own — listing it would offer the operator a
        // second, blurrier copy of every store.
        if (e.path().filename() == "overviews") {
          continue;
        }
        if (has_tif(e.path())) {
          const std::string sub_dir = e.path().string();
          const bool known = std::any_of(
            basemap_layers_.begin(), basemap_layers_.end(),
            [&sub_dir](const auto & l) {return l.second == sub_dir;});
          if (!known) {
            std::error_code rel_ec;
            const auto rel = std::filesystem::relative(e.path(), root, rel_ec);
            basemap_layers_.emplace_back(
              QString::fromStdString(rel_ec ? sub_dir : rel.string()), sub_dir);
          }
        }
        scan(e.path(), depth + 1);
      }
    };
  scan(std::filesystem::path(root), 1);
  // An explicit --stores dir that discovery didn't produce goes first (it was
  // asked for), labeled by its path.
  const bool initial_known = std::any_of(
    basemap_layers_.begin(), basemap_layers_.end(),
    [&initial_dir](const auto & l) {return l.second == initial_dir;});
  if (!initial_dir.empty() && !initial_known) {
    basemap_layers_.emplace(
      basemap_layers_.begin(),
      QString::fromStdString(initial_dir), initial_dir);
  }

  basemap_layer_->blockSignals(true);
  basemap_layer_->clear();
  int initial_index = 0;
  for (std::size_t i = 0; i < basemap_layers_.size(); ++i) {
    basemap_layer_->addItem(basemap_layers_[i].first);
    if (basemap_layers_[i].second == initial_dir) {
      initial_index = static_cast<int>(i);
    }
  }
  basemap_layer_->setCurrentIndex(initial_index);
  basemap_layer_->blockSignals(false);
  basemap_layer_->setVisible(basemap_layers_.size() > 1);
  basemap_cmap_->setVisible(!basemap_layers_.empty());
}

void SidescanViewerWindow::requestBasemapLoad()
{
  const int li = basemap_layer_ ? basemap_layer_->currentIndex() : -1;
  if (!bridge_ || li < 0 || li >= static_cast<int>(basemap_layers_.size())) {
    return;
  }
  const QString label = basemap_layers_[static_cast<std::size_t>(li)].first;
  const std::string dir = basemap_layers_[static_cast<std::size_t>(li)].second;
  // loadTile reads raw values; the uint16-backed sidescan composites use 0 as
  // their NoData sentinel (the float stores use NaN).
  const bool zero_is_nodata = dir.find("sidescan") != std::string::npos;
  const auto palette_idx = static_cast<std::size_t>(
    std::max(0, basemap_cmap_->currentIndex()));

  status_->setText(QString("Loading basemap %1…").arg(label));
  basemap_lod_->open(dir, palette_idx, zero_is_nodata);
}

void SidescanViewerWindow::pushBasemapView()
{
  if (!basemap_lod_ || !basemap_lod_->isOpen()) {
    return;
  }
  const auto region = canvas_->visibleGeoRegion();
  if (region) {
    basemap_lod_->viewChanged(*region, canvas_->groundMetresPerPixel());
  }
}

void SidescanViewerWindow::onTileSelectionChanged()
{
  if (!bridge_) {
    return;
  }
  std::vector<IndexedTile> selection;
  for (const auto idx : canvas_->selectedTiles()) {
    if (idx < indexed_tiles_.size()) {
      selection.push_back(indexed_tiles_[idx]);
    }
  }
  if (selection.empty()) {
    exitSelectionCloud();
    status_->setText("Tile selection cleared.");
    return;
  }

  std::vector<marine_survey_index::PassRow> rows;
  try {
    rows = bridge_->queryTiles(selection);
  } catch (const std::exception & e) {
    status_->setText(QString("Pass query failed: %1").arg(e.what()));
    return;
  }
  const auto passes = coalescePasses(rows);

  // The cloud loads every mbes-bathy pass of the selection, across bags —
  // sidescan stays single-pass by design (#258: blending kills shadows); its
  // passes cue the waterfall through the timeline. The timeline shows ALL
  // passes; an mbes bar carries its cloud-legend colour index so the two
  // panes correlate.
  std::vector<CloudPassInfo> cloud_passes;
  std::vector<TimelinePassInfo> timeline_passes;
  timeline_passes.reserve(passes.size());
  for (const auto & p : passes) {
    TimelinePassInfo bar;
    bar.bag_path = p.bag_path;
    bar.sensor_type = p.sensor_type;
    bar.t_start_ns = p.t_start_ns;
    bar.t_end_ns = p.t_end_ns;
    bar.ping_count = p.ping_count;
    if (p.sensor_type == "mbes-bathy") {
      bar.color_index = static_cast<int>(cloud_passes.size());
      CloudPassInfo info;
      info.bag_path = p.bag_path;
      info.t_start_ns = p.t_start_ns;
      info.t_end_ns = p.t_end_ns;
      info.label = passLabel(p.t_start_ns, p.bag_path);
      cloud_passes.push_back(std::move(info));
    }
    timeline_passes.push_back(std::move(bar));
  }
  selection_passes_ = timeline_passes;   // time->bag lookup for time-bar cues
  time_bar_->setPasses(std::move(timeline_passes));
  time_bar_->setVisible(true);

  // Selection mode: the cloud pane belongs to the selection until it clears.
  // A CUBE surface from a previous load is in a different reference frame —
  // drop it rather than draw it misplaced (#27).
  cube_surface_ = CubeSurface{};
  cube_drape_ = SidescanDrape{};
  cube_drape_terrain_ = CubeSurface{};
  cube_soundings_.clear();
  if (cube_selfcal_btn_) {cube_selfcal_btn_->setEnabled(false);}
  ++drape_gen_;
  cloud_->clearSurface();
  // Entering multi-pass mode from anything else defaults the colouring to
  // Pass once the load lands (#36); re-selecting while already in a
  // multi-pass cloud keeps the operator's choice. Either way the selector
  // stays live — Pass is an entry, not a mode that takes the control away,
  // which is what made a CUBE run look like it stole the cloud.
  cloud_pass_default_pending_ = !selection_cloud_;
  selection_cloud_ = true;
  // The pane is reloading: the passes on screen no longer answer to the
  // selection, and the frame they were loaded in is no longer current.
  cloud_pass_clouds_.clear();
  selection_ref_bag_.clear();
  selection_ref_frame_.clear();
  cloud_legend_->clear();
  cloud_legend_->setVisible(true);
  ++cloud_gen_;   // any load in flight is for a stale selection

  if (cloud_passes.empty()) {
    cloud_->resetView();
    cloud_->setMultiPassPoints({});
    cloud_frame_ = CloudFrame::None;   // an empty pane places nothing (#47)
    refreshCloudColorChannels();
    cloud_->setBoat(0.0, 0.0, 0.0, 0.0, false);
    status_->setText(
      QString("%1 tile%2 selected, %3 pass%4 — none mbes-bathy; nothing to "
        "load into the cloud.")
      .arg(selection.size()).arg(selection.size() == 1 ? "" : "s")
      .arg(passes.size()).arg(passes.size() == 1 ? "" : "es"));
    return;
  }

  // Contact clip (#24 desk finding): bound the cloud to the SELECTED
  // contact's neighbourhood when asked to.
  std::optional<GeoClip> clip;
  if (clip_contact_check_->isChecked()) {
    const int row = contact_list_->currentRow();
    if (row >= 0 && row < static_cast<int>(contact_store_.contacts().size())) {
      const auto & g = contact_store_.contacts()[static_cast<std::size_t>(row)]
        .geo_pose.position;
      if (std::isfinite(g.latitude) && std::isfinite(g.longitude)) {
        clip = GeoClip{g.latitude, g.longitude, 0.0, clip_margin_spin_->value()};
      }
    }
    if (!clip) {
      status_->setText(
        "Clip to contact: select a contact with a geo position first — "
        "loading unclipped.");
    }
  }

  cloud_passes_ = cloud_passes;
  status_->setText(
    QString("Loading %1 mbes-bathy pass%2 from %3 selected tile%4%5…")
    .arg(cloud_passes.size()).arg(cloud_passes.size() == 1 ? "" : "es")
    .arg(selection.size()).arg(selection.size() == 1 ? "" : "s")
    .arg(clip ? QString(" (clipped to contact, %1 m)").arg(clip->margin_m) :
    QString()));
  const auto gen = cloud_gen_;
  const auto snapshot = std::move(cloud_passes);   // worker owns its own copy
  const auto cancel = supersede_token(cloud_cancel_);
  cloud_watcher_.setFuture(QtConcurrent::run([snapshot, gen, clip, cancel]() {
      CloudLoadTicket ticket;
      ticket.generation = gen;
      // A non-QException escaping a QtConcurrent task std::terminates on Qt5,
      // and the throw resurfaces on the UI thread out of result() -- or out of
      // the destructor's waitForFinished(), i.e. a throw from a destructor.
      // The index and CUBE workers already guard; these two did not (#42
      // review). A multi-bag load is exactly where bad_alloc lives.
      try {
        ticket.outcome = load_cloud_passes(snapshot, clip, cancel);
      } catch (const std::exception & e) {
        ticket.outcome = CloudLoadOutcome();
        ticket.outcome.notes << QString("cloud load failed: %1").arg(e.what());
      } catch (...) {
        ticket.outcome = CloudLoadOutcome();
        ticket.outcome.notes << QString("cloud load failed: unknown exception");
      }
      return ticket;
    }));
}

std::string SidescanViewerWindow::passLabel(
  std::int64_t t_start_ns, const std::string & bag_path) const
{
  return QString("%1  (%2)")
         .arg(time_bar_->formatTime(t_start_ns))
         .arg(QFileInfo(QString::fromStdString(bag_path)).fileName())
         .toStdString();
}

void SidescanViewerWindow::refreshPassLabels()
{
  // The legend items bake formatted times at load; re-render them in the new
  // display zone. Counts (column 1) and check states are untouched.
  for (int i = 0; i < cloud_legend_->topLevelItemCount() &&
    i < static_cast<int>(cloud_passes_.size()); ++i)
  {
    auto & pass = cloud_passes_[static_cast<std::size_t>(i)];
    pass.label = passLabel(pass.t_start_ns, pass.bag_path);
    cloud_legend_->topLevelItem(i)->setText(0, QString::fromStdString(pass.label));
  }
}

void SidescanViewerWindow::exitSelectionCloud()
{
  selection_passes_.clear();
  cube_surface_ = CubeSurface{};   // the scrub cloud is a different frame (#27)
  cube_drape_ = SidescanDrape{};
  cube_drape_terrain_ = CubeSurface{};
  ++drape_gen_;
  cloud_->clearSurface();
  if (time_bar_) {
    time_bar_->clearPasses();
    // The extent survives the selection: the campaign (index mode), else the
    // open bag's span; with neither the bar has no extent and hides.
    if (!nav_track_points_.empty()) {
      std::int64_t t0 = nav_track_points_.front().t_ns;
      std::int64_t t1 = t0;
      for (const auto & p : nav_track_points_) {
        t0 = std::min(t0, p.t_ns);
        t1 = std::max(t1, p.t_ns);
      }
      time_bar_->setExtent(t0, t1);
    } else if (session_ && !loading_) {
      const double t0 = session_->timeAtDistance(0.0);
      const double t1 = session_->timeAtDistance(session_->totalDistance());
      time_bar_->setExtent(
        static_cast<std::int64_t>(t0 * 1e9), static_cast<std::int64_t>(t1 * 1e9));
    }
    time_bar_->setVisible(time_bar_->hasExtent());
  }
  if (!selection_cloud_) {
    return;
  }
  selection_cloud_ = false;
  ++cloud_gen_;   // an in-flight selection load must not apply any more
  cloud_pass_clouds_.clear();
  selection_ref_bag_.clear();
  selection_ref_frame_.clear();
  cloud_legend_->clear();
  cloud_legend_->setVisible(false);
  // Hand the pane back to the scrub window: re-render if a bag is open,
  // otherwise leave it empty.
  cloud_->resetView();
  if (session_) {
    requestRender();   // the render sets the pane's frame back to the open bag
  } else {
    cloud_->setPoints({});
    cloud_->setBoat(0.0, 0.0, 0.0, 0.0, false);
    cloud_frame_ = CloudFrame::None;
  }
  // The scrub cloud has no pass identity: Pass greys out (with its reason)
  // and the colouring falls back to Depth if it was the live choice.
  refreshCloudColorChannels();
}

void SidescanViewerWindow::onTimelinePassActivated(
  const QString & bag_path, qlonglong t_start_ns, qlonglong t_end_ns)
{
  const std::string bag = bag_path.toStdString();
  // Same bag, index complete: cue the scrub directly instead of re-opening
  // (an open re-runs the whole metadata index pass). Same trailing-window
  // head rule as the openBag cue.
  if (session_ && bag == current_bag_uri_ && !loading_) {
    const auto snapshot = session_->snapshot();
    const auto interval = snapshot ?
      distance_interval(
      *snapshot, static_cast<int64_t>(t_start_ns), static_cast<int64_t>(t_end_ns)) :
      std::nullopt;
    if (interval) {
      const double head = std::min(interval->first + window_len_m_, interval->second);
      scrub_->setValue(static_cast<int>(std::lround(head)));
      status_->setText(QString("Cued to pass %1–%2 m")
        .arg(interval->first, 0, 'f', 0).arg(interval->second, 0, 'f', 0));
    } else {
      status_->setText("Pass window matched no posed pings in the open bag.");
    }
    return;
  }
  openBag(
    bag, OpenReason::Cue,
    static_cast<int64_t>(t_start_ns), static_cast<int64_t>(t_end_ns));
}

void SidescanViewerWindow::onTimeSelected(qlonglong t_ns)
{
  // The operator committed a time on the time bar: cue there. A 5 s window
  // around the instant (the pass-coalescing gap scale) tolerates ping gaps.
  const auto t0 = static_cast<int64_t>(t_ns) - 2500000000LL;
  const auto t1 = static_cast<int64_t>(t_ns) + 2500000000LL;

  // Open bag first: a direct scrub jump beats a re-open.
  if (session_ && !loading_) {
    const auto snapshot = session_->snapshot();
    const auto interval =
      snapshot ? distance_interval(*snapshot, t0, t1) : std::nullopt;
    if (interval) {
      const double head = std::min(interval->first + window_len_m_, interval->second);
      scrub_->setValue(std::clamp(
          static_cast<int>(std::lround(head)), scrub_->minimum(), scrub_->maximum()));
      return;
    }
  }
  // While the current bag is still opening/indexing, a commit inside it
  // becomes the pending cue (applied when the index completes) instead of a
  // dead-end "No data" — fine-tuning during a load stays meaningful.
  const auto cue_into_loading_bag = [this, t0, t1, t_ns]() {
      pending_cue_start_ns_ = t0;
      pending_cue_end_ns_ = t1;
      status_->setText(QString("Will cue to %1 once %2 finishes indexing.")
        .arg(time_bar_->formatTime(static_cast<std::int64_t>(t_ns)))
        .arg(QFileInfo(QString::fromStdString(current_bag_uri_)).fileName()));
    };
  // Otherwise: the selection pass covering that time. The open bag is skipped
  // when idle (it just answered "nothing there"), but while loading it takes
  // the commit as the pending cue.
  for (const auto & p : selection_passes_) {
    if (t_ns >= p.t_start_ns && t_ns <= p.t_end_ns) {
      if (p.bag_path == current_bag_uri_) {
        if (loading_) {
          cue_into_loading_bag();
          return;
        }
        continue;
      }
      scheduleOpen(p.bag_path, t0, t1);
      return;
    }
  }
  // Otherwise: any campaign bag whose nav track covers that time — release
  // on the campaign-wide bar means "go there" (the bag-index cache makes the
  // open cheap after the first visit). Debounced: rapid fine-tune commits
  // collapse into one open.
  if (const auto fix = fixAtTime(nav_track_points_, static_cast<int64_t>(t_ns))) {
    for (const auto & [bag_id, path] : bag_paths_) {
      if (bag_id != fix->bag_id || path.empty()) {
        continue;
      }
      if (path == current_bag_uri_) {
        if (loading_) {
          cue_into_loading_bag();
          return;
        }
        continue;
      }
      scheduleOpen(path, t0, t1);
      return;
    }
  }
  status_->setText(
    QString("No data at %1 in the open bag or campaign.")
    .arg(time_bar_->formatTime(static_cast<std::int64_t>(t_ns))));
}

void SidescanViewerWindow::onCenterTimeChanged(qlonglong t_ns)
{
  // Live follower: the boat's interpolated fix at the bar's centre time, or
  // no arrow when the time falls between bags. Pure in-memory lookup, safe
  // at drag rates.
  const auto fix = fixAtTime(nav_track_points_, static_cast<int64_t>(t_ns));
  if (fix) {
    canvas_->setTimeArrow(
      SidescanCanvas::TimeArrow{fix->lat, fix->lon, fix->heading_rad});
  } else {
    canvas_->setTimeArrow(std::nullopt);
  }
}

void SidescanViewerWindow::onMapHoverGeo(double lat, double lon, bool valid)
{
  onPaneHoverGeo(
    HoverPane::Map,
    valid ? std::optional<GeoPoint>(GeoPoint{lat, lon}) : std::nullopt);
  if (!valid) {
    // No geographic position under the cursor means no fix search either:
    // the track is placed geographically, so there is nothing to search in.
    clearFixHighlight();
    return;
  }
  // No highlight while a gesture owns the pointer (#46): a region drag past
  // the slop, a pan, a glide, or contact marking. hoverGeo still fires
  // throughout those — the lat/lon readout should keep following the cursor —
  // so the suppression belongs here rather than at the emit.
  if (canvas_->pointerGestureActive()) {
    clearFixHighlight();
    return;
  }
  // In bag-only mode nav_track_points_ is empty, so this is a no-hit and the
  // feature is simply absent — no special case needed.
  const auto hit = nearestTrackFix(
    nav_track_points_, lat, lon, canvas_->groundMetresPerPixel());
  hovered_fix_ = hit;
  canvas_->setHighlightedFix(
    hit ?
    std::optional<SidescanCanvas::HighlightedFix>(
      SidescanCanvas::HighlightedFix{hit->latitude, hit->longitude}) :
    std::nullopt);
  refreshFixHighlightReadout();
}

void SidescanViewerWindow::onPaneHoverGeo(
  HoverPane pane, const std::optional<GeoPoint> & pos)
{
  hover_readout_.hover(pane, pos);
  hover_geo_->setText(QString::fromStdString(hover_readout_.text()));
}

void SidescanViewerWindow::onPaneHoverWorld(
  HoverPane pane, double map_x, double map_y, bool valid)
{
  // Each pane converts in ITS OWN frame: the three sonar panes hover in the
  // open bag's map-ENU, while the 3D pane's points may be a selection or CUBE
  // load in another bag's reference frame entirely.
  const std::optional<MapGeoAffine> anchor =
    (pane == HoverPane::Cloud) ? cloudFrameAnchor() : canvas_->mapAnchor();
  onPaneHoverGeo(
    pane, valid ? geo_from_map(anchor, map_x, map_y) : std::nullopt);
}

std::optional<MapGeoAffine> SidescanViewerWindow::cloudFrameAnchor() const
{
  switch (cloud_frame_) {
    case CloudFrame::OpenBag:
      // The scrub cloud is the open bag's window, in the same map-ENU the map
      // places that bag by — so it reads through the same anchor, live.
      return canvas_->mapAnchor();
    case CloudFrame::Reference:
      return cloud_ref_anchor_;
    case CloudFrame::None:
      break;
  }
  return std::nullopt;
}

void SidescanViewerWindow::onPaneLeave(HoverPane pane)
{
  hover_readout_.leave(pane);
  hover_geo_->setText(QString::fromStdString(hover_readout_.text()));
}

void SidescanViewerWindow::clearFixHighlight()
{
  hovered_fix_.reset();
  canvas_->setHighlightedFix(std::nullopt);
  hover_time_->clear();
}

void SidescanViewerWindow::refreshFixHighlightReadout()
{
  if (!hovered_fix_) {
    hover_time_->clear();
    return;
  }
  // The time bar's formatter, so this instant reads exactly as the same
  // instant does on the tape, in tooltips and in pass labels — and follows
  // the one UTC/local toggle rather than inventing a second answer.
  hover_time_->setText(time_bar_->formatTime(hovered_fix_->t_ns));
}

void SidescanViewerWindow::onMapPlainClicked()
{
  // With no highlighted fix a bare left click still does nothing at all —
  // #42's contract, and the reason it exists (a click that quietly destroyed
  // the operator's region) has not gone away. With one, the click cues.
  if (!hovered_fix_) {
    return;
  }
  // Deliberately the SAME path a committed time on the time bar takes: one
  // cueing implementation, so the map and the tape can never disagree about
  // what "go there" does. It resolves the recording itself, including the
  // reopen when the instant is in another one.
  onTimeSelected(static_cast<qlonglong>(hovered_fix_->t_ns));
}

void SidescanViewerWindow::onCloudPassesLoaded()
{
  // Non-const: the per-pass clouds are MOVED out below (a const ticket
  // silently degraded the move to a full copy — review round-2 finding).
  CloudLoadTicket ticket = cloud_watcher_.result();
  if (ticket.outcome.cancelled) {
    return;   // the window is closing (#44): the widgets below are going away
  }
  if (ticket.generation != cloud_gen_) {
    return;   // a newer selection (or a cleared one) superseded this load
  }
  const CloudLoadOutcome & out = ticket.outcome;

  cloud_->resetView();
  cloud_->setMultiPassPoints(out.pass_clouds);
  // The frame these soundings live in (#36): a later CUBE run may only lay a
  // surface over them when its own load resolved the same reference. It is
  // also the frame the pane's cursor reads out in (#47) — a load that
  // resolved no earth reference places nothing.
  selection_ref_bag_ = out.ref_bag;
  selection_ref_frame_ = out.ref_frame;
  cloud_frame_ = CloudFrame::Reference;
  cloud_ref_anchor_ =
    earthAnchorAffine(out.ref_earth_from_world, out.ref_has_geo, 0.0);
  // Pass identity exists again, so the Pass entry goes live; a selection that
  // just entered multi-pass mode also lands on it.
  refreshCloudColorChannels();
  if (cloud_pass_default_pending_) {
    cloud_pass_default_pending_ = false;
    for (int i = 0; i < cloud_color_combo_->count(); ++i) {
      if (channel_at(cloud_color_combo_, i) == ColorChannel::Pass) {
        cloud_color_combo_->setCurrentIndex(i);
        break;
      }
    }
  }
  // The selection cloud sits in the reference pass's world frame — the scrub
  // bag's boat arrow would be in the wrong frame, so hide it.
  cloud_->setBoat(0.0, 0.0, 0.0, 0.0, false);

  cloud_legend_->blockSignals(true);   // populating checkboxes is not a toggle
  cloud_legend_->clear();
  int total = 0;
  for (std::size_t i = 0; i < cloud_passes_.size(); ++i) {
    const int count = (i < out.sounding_counts.size()) ? out.sounding_counts[i] : 0;
    auto * item = new QTreeWidgetItem(cloud_legend_, {
        QString::fromStdString(cloud_passes_[i].label),
        QString::number(count)});
    QPixmap swatch(12, 12);
    float r = 1.0f;
    float g = 1.0f;
    float b = 1.0f;
    pass_color(static_cast<int>(i), r, g, b);
    swatch.fill(QColor::fromRgbF(r, g, b));
    item->setIcon(0, swatch);
    // Checkbox toggles this pass in the cloud (#24 desk ask).
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(0, Qt::Checked);
    if (count == 0) {
      item->setDisabled(true);
    }
    total += count;
  }
  cloud_legend_->blockSignals(false);
  // Keep the per-pass clouds so checkbox toggles re-filter without bag I/O.
  cloud_pass_clouds_ = std::move(ticket.outcome.pass_clouds);

  QString message = QString("%1 soundings from %2 pass%3")
    .arg(total).arg(cloud_passes_.size()).arg(cloud_passes_.size() == 1 ? "" : "es");
  if (cloud_->decimationStride() > 1) {
    message += QString(" — showing 1/%1 (render budget)")
      .arg(cloud_->decimationStride());
  }
  if (out.skipped_passes > 0) {
    message += QString(", %1 pass%2 skipped")
      .arg(out.skipped_passes).arg(out.skipped_passes == 1 ? "" : "es");
  }
  if (out.skipped_pings > 0) {
    message += QString(", %1 pings without TF").arg(out.skipped_pings);
  }
  if (!out.notes.isEmpty()) {
    // A many-pass clip can produce a note per pass; summarize past the
    // first few (the full list is not actionable from a status line).
    QStringList shown = out.notes.mid(0, 3);
    if (out.notes.size() > 3) {
      shown << QString("(+%1 more)").arg(out.notes.size() - 3);
    }
    message += " — " + shown.join("; ");
  }
  status_->setText(message);
  status_->setToolTip(out.notes.join("\n"));   // the full list, on hover
}

}  // namespace marine_perception_tools
