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

#include "sidescan_canvas.hpp"

#include <QColor>
#include <QGuiApplication>
#include <QFont>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <QTransform>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>
#include <vector>

#include "map_gesture.hpp"
#include "view_animation.hpp"

namespace marine_perception_tools
{

namespace
{

// Local-equirectangular metres per degree of latitude (mean Earth). The same
// constant the bridge's point query uses; display-grade, not geodetic.
constexpr double kMetersPerDegLat = 111320.0;

}  // namespace

SidescanCanvas::SidescanCanvas(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(480, 360);
  setMouseTracking(true);   // hover reporting (hoverWorld) needs moves without a button
  setAutoFillBackground(true);
  // Zoom settle: while the wheel is turning, repaints blit the stale layer
  // cache scaled; once it stops for a beat, one full-quality rebuild runs.
  cache_settle_.setSingleShot(true);
  cache_settle_.setInterval(160);
  connect(&cache_settle_, &QTimer::timeout, this, [this]() {
      cache_rebuild_due_ = true;
      update();
    });
  // Middle-click recentre glide (#42): ~60 Hz frames, but the position comes
  // from wall time (recenter_clock_), not from a frame count, so a dropped
  // frame costs smoothness and never the duration.
  recenter_timer_.setInterval(16);
  connect(&recenter_timer_, &QTimer::timeout, this, [this]() {stepRecenter();});
}

// --- middle-click recentre glide (#42) --------------------------------------

void SidescanCanvas::setRecenterDurationMs(int ms)
{
  recenter_duration_ms_ = std::max(0, ms);
  if (recentering_ && recenter_duration_ms_ == 0) {
    settleRecenter();   // switching to instant lands whatever is in flight
  }
}

void SidescanCanvas::startRecenter(const QPointF & target)
{
  recenter_to_ = target;
  user_adjusted_ = true;
  fit_pending_ = false;
  if (recenter_duration_ms_ <= 0) {
    // Instant path (headless snapshots, tests): identical outcome, no frames
    // in between for a capture to land on.
    recentering_ = false;
    recenter_timer_.stop();
    center_map_ = target;
    update();
    emit viewChanged();
    return;
  }
  // A second middle click retargets rather than cancelling: the gesture means
  // "go to here", so a fresh one means "actually, here" — and starting from
  // wherever the glide has reached keeps the map continuous, where cancelling
  // to a stop first would stutter. The full duration restarts, so the second
  // click reads exactly like the first.
  recenter_from_ = center_map_;
  recentering_ = true;
  recenter_clock_.start();
  recenter_timer_.start();
  update();
}

void SidescanCanvas::stepRecenter()
{
  if (!recentering_) {
    recenter_timer_.stop();
    return;
  }
  const double t = animationProgress(
    static_cast<double>(recenter_clock_.elapsed()),
    static_cast<double>(recenter_duration_ms_));
  if (t >= 1.0) {
    settleRecenter();
    return;
  }
  center_map_ = QPointF(
    easedInterpolate(recenter_from_.x(), recenter_to_.x(), t),
    easedInterpolate(recenter_from_.y(), recenter_to_.y(), t));
  update();   // repaints through the translated blit; the cache is untouched
}

void SidescanCanvas::settleRecenter()
{
  if (!recentering_) {
    return;
  }
  recenter_timer_.stop();
  recentering_ = false;
  center_map_ = recenter_to_;   // exact: the cache compares centres for equality
  update();   // the one full layer-cache rebuild for the whole glide
  emit viewChanged();
}

void SidescanCanvas::abandonRecenter()
{
  if (!recentering_) {
    return;
  }
  // The operator started something else. Stop where the glide stands and let
  // the new gesture own the view from there — landing on the old target
  // afterwards would move the map out from under them.
  recenter_timer_.stop();
  recentering_ = false;
}

void SidescanCanvas::finishRecenterNow()
{
  settleRecenter();
}

// --- geographic frame -------------------------------------------------------

void SidescanCanvas::setGeoOrigin(double lat_deg, double lon_deg)
{
  geo_mode_ = true;
  geo_lat0_ = lat_deg;
  geo_lon0_ = lon_deg;
  // Floored like GeoView::lonScale so a nonsensical polar origin cannot make
  // the plane non-invertible.
  lon_scale_ = std::max(0.01, std::cos(lat_deg * M_PI / 180.0));
  layer_cache_valid_ = false;
  rebuildGeoLayerGeometry();
  update();
}

void SidescanCanvas::setMapAnchor(const std::optional<MapGeoAffine> & anchor)
{
  map_anchor_ = anchor;
  update();
}

void SidescanCanvas::setStoreTiles(std::vector<OverviewTile> tiles)
{
  store_tiles_ = std::move(tiles);
  layer_cache_valid_ = false;
  rebuildGeoLayerGeometry();
  update();
}

void SidescanCanvas::setNavTrack(
  std::vector<std::vector<std::pair<double, double>>> segments)
{
  nav_segments_geo_ = std::move(segments);
  layer_cache_valid_ = false;
  rebuildGeoLayerGeometry();
  update();
}

void SidescanCanvas::setCoastline(Coastline coastline)
{
  coastline_geo_ = std::move(coastline);
  layer_cache_valid_ = false;
  rebuildGeoLayerGeometry();
  update();
}

void SidescanCanvas::setCoastlineVisible(bool on)
{
  if (show_coastline_ != on) {
    show_coastline_ = on;
    layer_cache_valid_ = false;
    update();
  }
}

void SidescanCanvas::setIndexTiles(const std::vector<GeoRect> & tiles)
{
  index_tiles_geo_ = tiles;
  layer_cache_valid_ = false;
  const bool had_selection = !selected_tiles_.empty();
  selected_tiles_.clear();
  rebuildGeoLayerGeometry();
  update();
  if (had_selection) {
    emit tileSelectionChanged();
  }
}

void SidescanCanvas::clearTileSelection()
{
  if (selected_tiles_.empty()) {
    return;
  }
  selected_tiles_.clear();
  update();
  emit tileSelectionChanged();
}

void SidescanCanvas::selectTiles(const std::set<std::size_t> & indices)
{
  std::set<std::size_t> valid;
  for (const auto idx : indices) {
    if (idx < index_tiles_.size()) {
      valid.insert(idx);
    }
  }
  if (valid == selected_tiles_) {
    return;
  }
  selected_tiles_ = std::move(valid);
  update();
  emit tileSelectionChanged();
}

void SidescanCanvas::setNavTrackVisible(bool on)
{
  if (show_nav_track_ != on) {
    show_nav_track_ = on;
    layer_cache_valid_ = false;
    update();
  }
}

void SidescanCanvas::setIndexTilesVisible(bool on)
{
  if (show_index_tiles_ != on) {
    show_index_tiles_ = on;
    layer_cache_valid_ = false;
    update();
  }
}

void SidescanCanvas::setMetricGridVisible(bool on)
{
  if (show_metric_grid_ != on) {
    show_metric_grid_ = on;
    layer_cache_valid_ = false;   // the grid lives in the cached static layer
    update();
  }
}

void SidescanCanvas::setTimeArrow(const std::optional<TimeArrow> & arrow)
{
  time_arrow_ = arrow;
  update();
}

void SidescanCanvas::fitGeo(double south, double west, double north, double east)
{
  abandonRecenter();   // an explicit fit replaces the view the glide was heading for
  fit_south_ = south;
  fit_west_ = west;
  fit_north_ = north;
  fit_east_ = east;
  fit_pending_ = true;
  user_adjusted_ = false;
  update();
}

// Local equirectangular about the geo origin. No antimeridian wrap handling:
// a survey straddling ±180° would split across the plane. That is a known,
// accepted restriction of the display-grade projection (the survey-index
// bridge's box queries already throw on antimeridian boxes — full split
// support is deferred deliberately, not overlooked).
QPointF SidescanCanvas::geoToCanvas(double lat, double lon) const
{
  return QPointF(
    (lon - geo_lon0_) * kMetersPerDegLat * lon_scale_,
    (lat - geo_lat0_) * kMetersPerDegLat);
}

std::pair<double, double> SidescanCanvas::canvasToGeo(double mx, double my) const
{
  return {
    geo_lat0_ + my / kMetersPerDegLat,
    geo_lon0_ + mx / (kMetersPerDegLat * lon_scale_)};
}

QPointF SidescanCanvas::bagToCanvas(double x, double y) const
{
  if (!geo_mode_) {
    return QPointF(x, y);
  }
  // Callers guard with mapPlaceable(); an anchorless call falls back to the
  // raw coordinates (never reached through the drawing paths).
  if (!map_anchor_) {
    return QPointF(x, y);
  }
  const auto & a = *map_anchor_;
  const double lat = a.lat0 + a.dlat_dx * x + a.dlat_dy * y;
  const double lon = a.lon0 + a.dlon_dx * x + a.dlon_dy * y;
  return geoToCanvas(lat, lon);
}

std::optional<QPointF> SidescanCanvas::canvasToBag(double mx, double my) const
{
  if (!geo_mode_) {
    return QPointF(mx, my);
  }
  if (!map_anchor_) {
    return std::nullopt;
  }
  const auto & a = *map_anchor_;
  const auto geo = canvasToGeo(mx, my);
  const double dlat = geo.first - a.lat0;
  const double dlon = geo.second - a.lon0;
  const double det = a.dlat_dx * a.dlon_dy - a.dlat_dy * a.dlon_dx;
  if (std::abs(det) < 1e-18) {
    return std::nullopt;
  }
  return QPointF(
    (dlat * a.dlon_dy - a.dlat_dy * dlon) / det,
    (a.dlat_dx * dlon - dlat * a.dlon_dx) / det);
}

void SidescanCanvas::rebuildGeoLayerGeometry()
{
  store_tile_rects_.clear();
  nav_segments_.clear();
  index_tiles_.clear();
  coastline_.clear();
  coastline_bounds_.clear();
  if (!geo_mode_) {
    return;
  }
  store_tile_rects_.reserve(store_tiles_.size());
  for (const auto & tile : store_tiles_) {
    const QPointF sw = geoToCanvas(tile.south, tile.west);
    const QPointF ne = geoToCanvas(tile.north, tile.east);
    store_tile_rects_.emplace_back(
      sw.x(), sw.y(), ne.x() - sw.x(), ne.y() - sw.y());
  }
  coastline_.clear();
  coastline_bounds_.clear();
  coastline_.reserve(coastline_geo_.lines.size());
  coastline_bounds_.reserve(coastline_geo_.lines.size());
  for (const auto & line : coastline_geo_.lines) {
    QPolygonF poly;
    poly.reserve(static_cast<int>(line.points.size()));
    for (const auto & [lat, lon] : line.points) {
      poly << geoToCanvas(lat, lon);
    }
    const QPointF sw = geoToCanvas(line.south, line.west);
    const QPointF ne = geoToCanvas(line.north, line.east);
    coastline_bounds_.emplace_back(
      sw.x(), sw.y(), ne.x() - sw.x(), ne.y() - sw.y());
    coastline_.push_back(std::move(poly));
  }
  nav_segments_.reserve(nav_segments_geo_.size());
  for (const auto & seg : nav_segments_geo_) {
    QPolygonF poly;
    poly.reserve(static_cast<int>(seg.size()));
    for (const auto & [lat, lon] : seg) {
      poly << geoToCanvas(lat, lon);
    }
    nav_segments_.push_back(std::move(poly));
  }
  index_tiles_.reserve(index_tiles_geo_.size());
  for (const auto & tile : index_tiles_geo_) {
    const QPointF sw = geoToCanvas(tile.south, tile.west);
    const QPointF ne = geoToCanvas(tile.north, tile.east);
    index_tiles_.push_back(SelectableRect{sw.x(), sw.y(), ne.x(), ne.y()});
  }
}

void SidescanCanvas::applyPendingFit()
{
  if (!fit_pending_ || user_adjusted_ || width() <= 0 || height() <= 0) {
    return;
  }
  const QPointF sw = geoToCanvas(fit_south_, fit_west_);
  const QPointF ne = geoToCanvas(fit_north_, fit_east_);
  center_map_ = QPointF((sw.x() + ne.x()) * 0.5, (sw.y() + ne.y()) * 0.5);
  const double ext_x = std::max(1.0, ne.x() - sw.x());
  const double ext_y = std::max(1.0, ne.y() - sw.y());
  const double fit = std::min(width() / ext_x, height() / ext_y);
  px_per_m_ = std::clamp((fit > 0.0 ? fit : 4.0) * 0.95, 1e-4, 500.0);
  // Stay pending: refit on every resize until the user takes the view over,
  // so the first laid-out paint (not the pre-layout ctor size) wins.
  emit viewChanged();
}

std::optional<GeoRect> SidescanCanvas::visibleGeoRegion() const
{
  if (!geo_mode_ || width() <= 0 || height() <= 0) {
    return std::nullopt;
  }
  const QPointF tl = screenToMap(0.0, 0.0);
  const QPointF br = screenToMap(width(), height());
  const auto [n_lat, w_lon] = canvasToGeo(tl.x(), tl.y());
  const auto [s_lat, e_lon] = canvasToGeo(br.x(), br.y());
  return GeoRect{
    std::min(s_lat, n_lat), std::min(w_lon, e_lon),
    std::max(s_lat, n_lat), std::max(w_lon, e_lon)};
}

// --- per-bag layers ---------------------------------------------------------

void SidescanCanvas::setCoverage(
  const QImage & image, double origin_x, double origin_y, double res_m)
{
  coverage_ = image;
  cov_origin_x_ = origin_x;
  cov_origin_y_ = origin_y;
  cov_res_m_ = (res_m > 0.0) ? res_m : 1.0;
  have_coverage_ = !image.isNull();
  update();
}

void SidescanCanvas::setTrack(const std::vector<QPointF> & track_map)
{
  track_ = track_map;
  update();
}

void SidescanCanvas::setGridSpacing(double metres)
{
  if (metres > 0.0) {
    grid_spacing_m_ = metres;
    layer_cache_valid_ = false;
    update();
  }
}

void SidescanCanvas::setCenter(double map_x, double map_y)
{
  if (!mapPlaceable()) {
    return;
  }
  abandonRecenter();   // following the playhead outranks a glide already running
  center_map_ = bagToCanvas(map_x, map_y);
  update();
}

void SidescanCanvas::setMarkMode(bool on)
{
  mark_mode_ = on;
  setCursor(on ? Qt::CrossCursor : Qt::ArrowCursor);
}

void SidescanCanvas::setCursorWorld(const std::optional<QPointF> & map_point)
{
  cursor_world_ = map_point;
  update();
}

void SidescanCanvas::setContacts(const QVector<ContactMarker> & contacts)
{
  contacts_ = contacts;
  update();
}

QPointF SidescanCanvas::mapToScreen(double mx, double my) const
{
  // North-up: +x east -> right, +y north -> up (screen y grows downward).
  const double sx = width() * 0.5 + (mx - center_map_.x()) * px_per_m_;
  const double sy = height() * 0.5 - (my - center_map_.y()) * px_per_m_;
  return QPointF(sx, sy);
}

QPointF SidescanCanvas::screenToMap(double sx, double sy) const
{
  const double mx = center_map_.x() + (sx - width() * 0.5) / px_per_m_;
  const double my = center_map_.y() - (sy - height() * 0.5) / px_per_m_;
  return QPointF(mx, my);
}

void SidescanCanvas::resetView()
{
  abandonRecenter();
  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
  bool have = false;
  if (mapPlaceable() && have_coverage_) {
    const QPointF sw = bagToCanvas(cov_origin_x_, cov_origin_y_);
    const QPointF ne = bagToCanvas(
      cov_origin_x_ + coverage_.width() * cov_res_m_,
      cov_origin_y_ + coverage_.height() * cov_res_m_);
    min_x = std::min(sw.x(), ne.x());
    max_x = std::max(sw.x(), ne.x());
    min_y = std::min(sw.y(), ne.y());
    max_y = std::max(sw.y(), ne.y());
    have = true;
  } else if (mapPlaceable() && !track_.empty()) {
    const QPointF first = bagToCanvas(track_.front().x(), track_.front().y());
    min_x = max_x = first.x();
    min_y = max_y = first.y();
    for (const auto & p : track_) {
      const QPointF c = bagToCanvas(p.x(), p.y());
      min_x = std::min(min_x, c.x());
      max_x = std::max(max_x, c.x());
      min_y = std::min(min_y, c.y());
      max_y = std::max(max_y, c.y());
    }
    have = true;
  }
  if (!have) {
    if (geo_mode_ && (fit_pending_ || fit_north_ > fit_south_)) {
      // No bag content: refit the survey bounds instead.
      fit_pending_ = true;
      user_adjusted_ = false;
      update();
    }
    return;
  }
  user_adjusted_ = true;   // an explicit bag fit overrides the pending geo fit
  fit_pending_ = false;
  center_map_ = QPointF((min_x + max_x) * 0.5, (min_y + max_y) * 0.5);
  const double ext_x = std::max(1.0, max_x - min_x);
  const double ext_y = std::max(1.0, max_y - min_y);
  const double fit = std::min(width() / ext_x, height() / ext_y);
  px_per_m_ = (fit > 0.0 ? fit : 4.0) * 0.9;
  update();
}

void SidescanCanvas::drawGrid(QPainter & painter) const
{
  if (!show_metric_grid_) {
    return;
  }
  const QPointF tl = screenToMap(0, 0);
  const QPointF br = screenToMap(width(), height());
  const double min_x = std::min(tl.x(), br.x());
  const double max_x = std::max(tl.x(), br.x());
  const double min_y = std::min(tl.y(), br.y());
  const double max_y = std::max(tl.y(), br.y());

  // Don't draw an unreadable mass of lines when zoomed far out.
  if ((max_x - min_x) / grid_spacing_m_ > 400.0) {return;}

  QPen pen(QColor(80, 90, 100));
  pen.setWidth(0);
  painter.setPen(pen);
  painter.setFont(QFont(painter.font().family(), 7));

  const double s = grid_spacing_m_;
  for (double x = std::ceil(min_x / s) * s; x <= max_x; x += s) {
    const QPointF a = mapToScreen(x, min_y);
    const QPointF b = mapToScreen(x, max_y);
    painter.drawLine(a, b);
    painter.drawText(QPointF(a.x() + 2, height() - 3), QString::number(x, 'f', 0));
  }
  for (double y = std::ceil(min_y / s) * s; y <= max_y; y += s) {
    const QPointF a = mapToScreen(min_x, y);
    const QPointF b = mapToScreen(max_x, y);
    painter.drawLine(a, b);
    painter.drawText(QPointF(2, a.y() - 2), QString::number(y, 'f', 0));
  }
}

void SidescanCanvas::drawCoastline(QPainter & painter) const
{
  if (coastline_.empty() || !show_coastline_) {
    return;
  }
  // The scale rule (coastline_data.hpp): full strength where nothing else
  // tells the operator where they are, gone before survey zoom. This is the
  // whole reason the layer is safe to ship — a generalised coastline that
  // stayed visible while the operator zoomed into a survey would read as
  // chart detail and be wrong by hundreds of metres.
  const double fade = coastlineFadeAlpha(groundMetresPerPixel());
  if (fade <= 0.0) {
    return;
  }
  // Desaturated slate, kept off the blue-green end the depth colormaps own,
  // and never brighter than the nav track's veil.
  QPen pen(QColor(150, 165, 180, static_cast<int>(std::lround(150.0 * fade))));
  pen.setWidthF(1.0);
  painter.setPen(pen);
  painter.setBrush(Qt::NoBrush);
  const QRectF viewport(0, 0, width(), height());
  for (std::size_t i = 0; i < coastline_.size(); ++i) {
    // Cull by the polyline's own bounds first: most of the world is off
    // screen at every zoom the layer is drawn at.
    const auto & b = coastline_bounds_[i];
    const QPointF nw = mapToScreen(b.left(), b.top() + b.height());
    const QPointF se = mapToScreen(b.left() + b.width(), b.top());
    if (!QRectF(nw, se).normalized().adjusted(-2, -2, 2, 2).intersects(viewport)) {
      continue;
    }
    // Sparse, as for the nav track: skip points that advance the polyline by
    // less than ~2 screen px (60k world points collapse to a few thousand).
    const auto & seg = coastline_[i];
    if (seg.size() < 2) {
      continue;
    }
    QPolygonF screen;
    screen.reserve(seg.size());
    QPointF last = mapToScreen(seg.front().x(), seg.front().y());
    screen << last;
    for (int j = 1; j < seg.size(); ++j) {
      const QPointF pt = mapToScreen(seg[j].x(), seg[j].y());
      if (j == seg.size() - 1 ||
        std::abs(pt.x() - last.x()) + std::abs(pt.y() - last.y()) >= 2.0)
      {
        screen << pt;
        last = pt;
      }
    }
    painter.drawPolyline(screen);
  }
}

void SidescanCanvas::drawNavTrack(QPainter & painter) const
{
  if (nav_segments_.empty() || !show_nav_track_) {
    return;
  }
  // Light-handed: a campaign's worth of overlapping passes must read as a
  // veil over the basemap, not a blanket (and the toggle removes it wholly).
  // No per-track arrowheads — the time-bar arrow gives direction on demand.
  // Sparse: skip points that advance the polyline by less than ~2 screen px
  // (46.5k campaign points collapse to a few thousand at survey zooms).
  QPen pen(QColor(255, 255, 255, 90));
  pen.setWidthF(1.0);
  painter.setPen(pen);
  painter.setBrush(Qt::NoBrush);
  for (const auto & seg : nav_segments_) {
    if (seg.size() < 2) {
      continue;
    }
    QPolygonF screen;
    screen.reserve(seg.size());
    QPointF last = mapToScreen(seg.front().x(), seg.front().y());
    screen << last;
    for (int i = 1; i < seg.size(); ++i) {
      const QPointF pt = mapToScreen(seg[i].x(), seg[i].y());
      if (i == seg.size() - 1 ||
        std::abs(pt.x() - last.x()) + std::abs(pt.y() - last.y()) >= 2.0)
      {
        screen << pt;
        last = pt;
      }
    }
    painter.drawPolyline(screen);
  }
}

void SidescanCanvas::drawIndexTileGrid(QPainter & painter) const
{
  if (index_tiles_.empty() || !show_index_tiles_) {
    return;
  }
  // Sparse: below ~3 px per tile the outlines are unreadable haze — skip.
  if ((index_tiles_.front().x1 - index_tiles_.front().x0) * px_per_m_ < 3.0) {
    return;
  }
  const QRectF viewport(0, 0, width(), height());
  QPen outline(QColor(0, 200, 255, 45));
  outline.setWidthF(1.0);
  painter.setPen(outline);
  painter.setBrush(Qt::NoBrush);
  for (const auto & t : index_tiles_) {
    const QPointF nw = mapToScreen(t.x0, t.y1);
    const QPointF se = mapToScreen(t.x1, t.y0);
    const QRectF r(nw, se);
    if (r.intersects(viewport)) {
      painter.drawRect(r);
    }
  }
}

void SidescanCanvas::drawSelectedTiles(QPainter & painter) const
{
  if (selected_tiles_.empty()) {
    return;
  }
  const QRectF viewport(0, 0, width(), height());
  painter.setPen(QPen(QColor(0, 255, 255, 220), 1.5));
  painter.setBrush(QColor(0, 255, 255, 70));
  for (const auto i : selected_tiles_) {
    if (i >= index_tiles_.size()) {
      continue;
    }
    const auto & t = index_tiles_[i];
    const QPointF nw = mapToScreen(t.x0, t.y1);
    const QPointF se = mapToScreen(t.x1, t.y0);
    const QRectF r(nw, se);
    if (r.intersects(viewport)) {
      painter.drawRect(r);
    }
  }
  painter.setBrush(Qt::NoBrush);
}

void SidescanCanvas::rebuildLayerCache()
{
  if (size().isEmpty()) {
    return;   // pre-layout paint: nothing sane to rasterize yet
  }
  ++layer_cache_rebuilds_;
  layer_cache_ = QPixmap(size());
  QPainter painter(&layer_cache_);
  painter.fillRect(layer_cache_.rect(), QColor(20, 24, 28));

  // Bottom of the stack, under the store basemap: an orientation layer must
  // never occlude real data (#41).
  if (geo_mode_) {
    drawCoastline(painter);
  }

  if (geo_mode_ && !store_tile_rects_.empty()) {
    // Nearest-neighbour on purpose: store cells must stay crisp pixels so
    // the imagery reads at its true resolution — never bilinear-blended.
    for (std::size_t i = 0; i < store_tile_rects_.size(); ++i) {
      const auto & r = store_tile_rects_[i];
      const QPointF nw = mapToScreen(r.left(), r.top() + r.height());
      const QPointF se = mapToScreen(r.left() + r.width(), r.top());
      const QRectF target(nw, se);
      if (!target.intersects(layer_cache_.rect())) {
        continue;
      }
      painter.drawImage(target, store_tiles_[i].image);
    }
  }

  drawGrid(painter);
  if (geo_mode_) {
    drawNavTrack(painter);
    drawIndexTileGrid(painter);
  }

  layer_cache_valid_ = true;
  cache_px_per_m_ = px_per_m_;
  cache_center_ = center_map_;
  cache_size_ = size();
}

void SidescanCanvas::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  QPainter painter(this);
  applyPendingFit();

  // Static layers from the cache (see rebuildLayerCache): a same-view repaint
  // is a blit; a mid-pan repaint blits the stale cache translated and rebuilds
  // on release; anything else (zoom, resize, data change) rebuilds now.
  // A lost middle-button release (modal mid-drag, grab stolen) must not pin
  // us on the translated-blit branch forever (review round-2 finding). The
  // button is the middle one since #42 moved panning off left-drag.
  if (panning_ && !(QGuiApplication::mouseButtons() & Qt::MiddleButton)) {
    panning_ = false;
  }
  const bool view_matches = layer_cache_valid_ && cache_size_ == size() &&
    cache_px_per_m_ == px_per_m_ && cache_center_ == center_map_;
  const bool stale_ok = layer_cache_valid_ && cache_size_ == size() &&
    !cache_rebuild_due_;
  // A recentre glide moves the centre every frame for three quarters of a
  // second (#42). It rides the same translated blit as a pan for exactly the
  // reason the pan does: a per-frame rebuild of the coastline, basemap and
  // tile grid would stutter on a collection-wide view. One rebuild happens
  // when it settles, on the frame after recentering_ goes false.
  const bool pan_blit = (panning_ || recentering_) && stale_ok &&
    cache_px_per_m_ == px_per_m_;
  // A zoom step (wheel) blits the stale cache scaled about the widget centre
  // — same slippy-map idea as the pan blit — and queues one full-quality
  // rebuild for when the wheel settles (#26 snappiness).
  const bool zoom_blit = stale_ok && cache_px_per_m_ > 0.0 &&
    cache_px_per_m_ != px_per_m_;
  if (view_matches) {
    cache_rebuild_due_ = false;   // the view came back; the cache is exact
    painter.drawPixmap(0, 0, layer_cache_);
  } else if (pan_blit) {
    painter.fillRect(rect(), QColor(20, 24, 28));
    const QPointF off(
      (cache_center_.x() - center_map_.x()) * px_per_m_,
      (center_map_.y() - cache_center_.y()) * px_per_m_);
    painter.drawPixmap(off, layer_cache_);
  } else if (zoom_blit) {
    painter.fillRect(rect(), QColor(20, 24, 28));
    // Cache pixel p was rendered at the old view; under the new one the same
    // map point lands at C + s*(p - C) + (cache_center - center)*px_per_m
    // (y flipped) with s the zoom ratio — a pure transform blit.
    const double s = px_per_m_ / cache_px_per_m_;
    const QPointF c(width() * 0.5, height() * 0.5);
    QTransform t;
    t.translate(
      c.x() + (cache_center_.x() - center_map_.x()) * px_per_m_,
      c.y() + (center_map_.y() - cache_center_.y()) * px_per_m_);
    t.scale(s, s);
    t.translate(-c.x(), -c.y());
    painter.setTransform(t);
    painter.drawPixmap(0, 0, layer_cache_);
    painter.resetTransform();
    cache_settle_.start();   // one real rebuild once the wheel stops
  } else {
    rebuildLayerCache();
    cache_rebuild_due_ = false;
    painter.drawPixmap(0, 0, layer_cache_);
  }

  // --- dynamic overlays, cheap per frame ---
  if (geo_mode_) {
    drawSelectedTiles(painter);
  }

  const bool bag_placeable = mapPlaceable();
  if (have_coverage_ && bag_placeable) {
    // Image extent in map frame: SW corner (origin) to NE corner; the image is
    // north-up (row 0 = north). Place its corners through the bag anchor and
    // let the painter transform absorb the (near-identity) affine exactly.
    const double w_m = coverage_.width() * cov_res_m_;
    const double h_m = coverage_.height() * cov_res_m_;
    const QPointF nw = bagToCanvas(cov_origin_x_, cov_origin_y_ + h_m);
    const QPointF ne = bagToCanvas(cov_origin_x_ + w_m, cov_origin_y_ + h_m);
    const QPointF sw = bagToCanvas(cov_origin_x_, cov_origin_y_);
    const QPointF s_nw = mapToScreen(nw.x(), nw.y());
    const QPointF s_ne = mapToScreen(ne.x(), ne.y());
    const QPointF s_sw = mapToScreen(sw.x(), sw.y());
    QTransform t;
    // Image pixel (u, v) -> screen: origin s_nw, u along (s_ne-s_nw)/w_px,
    // v along (s_sw-s_nw)/h_px.
    const double iw = std::max(1, coverage_.width());
    const double ih = std::max(1, coverage_.height());
    t.setMatrix(
      (s_ne.x() - s_nw.x()) / iw, (s_ne.y() - s_nw.y()) / iw, 0.0,
      (s_sw.x() - s_nw.x()) / ih, (s_sw.y() - s_nw.y()) / ih, 0.0,
      s_nw.x(), s_nw.y(), 1.0);
    painter.save();
    painter.setTransform(t);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
    painter.drawImage(QPointF(0, 0), coverage_);
    painter.restore();
  }

  if (track_.size() > 1 && bag_placeable) {
    QPen pen(QColor(120, 200, 255, 180));
    pen.setWidthF(1.5);
    painter.setPen(pen);
    QPolygonF poly;
    poly.reserve(static_cast<int>(track_.size()));
    for (const auto & p : track_) {
      const QPointF c = bagToCanvas(p.x(), p.y());
      poly << mapToScreen(c.x(), c.y());
    }
    painter.drawPolyline(poly);
  }

  // Contact markers (map-anchored), drawn wherever they fall in the current view.
  if (!contacts_.empty() && bag_placeable) {
    QPen pen(QColor(255, 0, 255));
    pen.setWidthF(1.5);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    for (const auto & c : contacts_) {
      const QPointF cnw = bagToCanvas(c.x - 0.5 * c.w, c.y + 0.5 * c.h);
      const QPointF nw = mapToScreen(cnw.x(), cnw.y());
      const double wpx = std::max(6.0, c.w * px_per_m_);
      const double hpx = std::max(6.0, c.h * px_per_m_);
      const QRectF r(nw, QSizeF(wpx, hpx));
      painter.drawRect(r);
      if (!c.id.isEmpty()) {
        painter.drawText(QPointF(r.left(), r.top() - 2), c.id);
      }
    }
  }

  // Time-bar position arrow: the boat at the bar's centre time, in the 3D
  // pane's boat-arrow orange for consistent iconography. Drawn near the TOP
  // of the stack — it is a transient indicator and must never hide under the
  // coverage imagery or other layers (desk-verify finding).
  if (geo_mode_ && time_arrow_) {
    const QPointF c = geoToCanvas(time_arrow_->lat, time_arrow_->lon);
    const QPointF s = mapToScreen(c.x(), c.y());
    // Heading is CW from north; screen y grows downward, so the north-up
    // rotation is the same angle about the screen point.
    const double a = time_arrow_->heading_rad;
    const double ca = std::cos(a);
    const double sa = std::sin(a);
    const auto rot = [&](double fwd, double right) {
        // forward = north(-y on screen), right = east(+x on screen).
        return s + QPointF(
          right * ca + fwd * sa,
          right * sa - fwd * ca);
      };
    constexpr double kL = 12.0;   // arrow length, px
    QPolygonF arrow;
    arrow << rot(kL, 0.0) << rot(-0.5 * kL, 0.55 * kL)
          << rot(-0.2 * kL, 0.0) << rot(-0.5 * kL, -0.55 * kL);
    painter.setPen(QPen(QColor(20, 20, 20), 1.0));
    painter.setBrush(QColor(255, 140, 0));   // boat-arrow orange
    painter.drawPolygon(arrow);
    painter.setBrush(Qt::NoBrush);
  }

  // Cross-pane linked cursor (cyan cross at the shared map point).
  if (cursor_world_.has_value() && bag_placeable) {
    const QPointF cc = bagToCanvas(cursor_world_->x(), cursor_world_->y());
    const QPointF c = mapToScreen(cc.x(), cc.y());
    QPen pen(QColor(0, 255, 255));
    pen.setWidthF(1.5);
    painter.setPen(pen);
    const double s = 7.0;
    painter.drawLine(QPointF(c.x() - s, c.y()), QPointF(c.x() + s, c.y()));
    painter.drawLine(QPointF(c.x(), c.y() - s), QPointF(c.x(), c.y() + s));
  }

  // Rubber-band box while drawing a contact.
  if (marking_) {
    QPen pen(QColor(255, 0, 255));
    pen.setStyle(Qt::DashLine);
    painter.setPen(pen);
    painter.setBrush(QColor(255, 0, 255, 40));
    painter.drawRect(QRectF(mark_start_, mark_cur_).normalized());
  }

  // The region (#42): the persistent geographic rectangle every action reads.
  // Orange like the time arrow — the "lab focus" accents.
  if (geo_mode_ && cube_box_geo_) {
    const QPointF sw = geoToCanvas(cube_box_geo_->south, cube_box_geo_->west);
    const QPointF ne = geoToCanvas(cube_box_geo_->north, cube_box_geo_->east);
    const QPointF s_sw = mapToScreen(sw.x(), sw.y());
    const QPointF s_ne = mapToScreen(ne.x(), ne.y());
    QPen pen(QColor(255, 140, 0));
    pen.setWidthF(1.5);
    painter.setPen(pen);
    painter.setBrush(QColor(255, 140, 0, 25));
    painter.drawRect(QRectF(s_sw, s_ne).normalized());
  }
  // Live rubber band while the region is being dragged. Dashed, so a drag in
  // progress never looks like a settled selection.
  if (region_selecting_) {
    QPen pen(QColor(255, 140, 0));
    pen.setStyle(Qt::DashLine);
    painter.setPen(pen);
    painter.setBrush(QColor(255, 140, 0, 30));
    painter.drawRect(QRectF(region_start_, region_cur_).normalized());
  }
}

void SidescanCanvas::resizeEvent(QResizeEvent * event)
{
  QWidget::resizeEvent(event);
  // A pending geo fit re-applies at the new size until the user takes over —
  // the first real fit must wait for the laid-out size.
  if (fit_pending_ && !user_adjusted_) {
    update();
  }
  emit viewChanged();
}

void SidescanCanvas::wheelEvent(QWheelEvent * event)
{
  const double steps = event->angleDelta().y() / 120.0;
  if (steps == 0.0) {return;}
  abandonRecenter();   // zooming takes the view over from here, mid-glide
  const QPointF cursor = event->position();
  const QPointF before = screenToMap(cursor.x(), cursor.y());
  const double factor = std::pow(1.2, steps);
  px_per_m_ = std::clamp(px_per_m_ * factor, 1e-4, 500.0);
  // Keep the map point under the cursor fixed.
  const QPointF after = screenToMap(cursor.x(), cursor.y());
  center_map_ += before - after;
  user_adjusted_ = true;
  fit_pending_ = false;
  update();
  emit viewChanged();
}

void SidescanCanvas::mousePressEvent(QMouseEvent * event)
{
  // The middle button carries both halves of "go to here" (#42, the GeoZui
  // idiom): press-and-release centres the view on the point, press-and-drag
  // pans. Which one it was is only known at release, so the decision waits
  // there and nothing is committed on press.
  // Any new press ends a recentre glide in flight (#42): a middle press is
  // about to pan or retarget, and a left press draws a region whose corners
  // are read in screen pixels — the map must not slide out from under it.
  abandonRecenter();
  if (event->button() == Qt::MiddleButton) {
    middle_start_ = event->pos();
    last_drag_pos_ = event->pos();
    middle_dragging_ = true;
    middle_moved_ = false;
    return;
  }
  if (event->button() != Qt::LeftButton) {return;}
  // Contact marking is an explicitly chosen, visible mode, so it outranks the
  // default left-drag gesture while it is on.
  if (mark_mode_) {
    marking_ = true;
    mark_start_ = event->pos();
    mark_cur_ = event->pos();
    update();
    return;
  }
  // Left drag draws the map's one region (#42). Ctrl and Shift no longer mean
  // anything here: the region replaced both the index-tile rubber band and the
  // separate CUBE box, so one rectangle feeds every action.
  region_selecting_ = true;
  region_start_ = event->pos();
  region_cur_ = event->pos();
  update();
}

void SidescanCanvas::mouseMoveEvent(QMouseEvent * event)
{
  // Report the hovered position for the cross-pane linked cursor (map frame,
  // suppressed when the bag is not placeable) and the status readout (geo).
  const QPointF hov = screenToMap(event->pos().x(), event->pos().y());
  const auto bag = canvasToBag(hov.x(), hov.y());
  if (bag) {
    emit hoverWorld(bag->x(), bag->y(), true);
  } else {
    emit hoverWorld(0.0, 0.0, false);
  }
  if (geo_mode_) {
    const auto geo = canvasToGeo(hov.x(), hov.y());
    emit hoverGeo(geo.first, geo.second);
  }
  // Panning moved to the middle button (#42), because left-drag now draws the
  // region. A middle drag that has travelled past the click threshold is a
  // pan, and having moved at all is what stops the release from centring.
  if ((event->buttons() & Qt::MiddleButton) && middle_dragging_) {
    if (!middle_moved_ &&
      (event->pos() - middle_start_).manhattanLength() > kClickSlopPx)
    {
      middle_moved_ = true;
    }
    if (middle_moved_) {
      panning_ = true;   // repaint via the translated cache until release
      const QPoint delta = event->pos() - last_drag_pos_;
      last_drag_pos_ = event->pos();
      center_map_ += QPointF(-delta.x() / px_per_m_, delta.y() / px_per_m_);
      user_adjusted_ = true;
      fit_pending_ = false;
      update();
    }
    return;
  }
  if (!(event->buttons() & Qt::LeftButton)) {return;}
  if (region_selecting_) {
    region_cur_ = event->pos();
    update();
    return;
  }
  if (marking_) {
    mark_cur_ = event->pos();
    update();
    return;
  }
}

void SidescanCanvas::mouseReleaseEvent(QMouseEvent * event)
{
  // Middle release: a pan settles, a click centres. Centring keeps the seek
  // that this gesture already performed in every other pane (#42) — the two
  // compose as "go to here", spatially and, with a bag open, in time.
  if (event->button() == Qt::MiddleButton && middle_dragging_) {
    middle_dragging_ = false;
    if (middle_moved_) {
      panning_ = false;
      update();   // rebuild the layer cache at the settled view
      emit viewChanged();
      return;
    }
    const QPointF m = screenToMap(event->pos().x(), event->pos().y());
    // The view glides to the point over about three quarters of a second so
    // the operator can see where the map went (startRecenter emits
    // viewChanged when it settles, or immediately on the instant path).
    startRecenter(m);
    // The seek fires at the CLICK, not at the landing: the time cursor must
    // not lag the pointer by the length of the animation.
    const auto bag = canvasToBag(m.x(), m.y());
    if (bag) {
      emit seekWorld(bag->x(), bag->y());
    }
    return;
  }
  if (event->button() != Qt::LeftButton) {return;}
  if (region_selecting_) {
    region_selecting_ = false;
    const bool was_click =
      (event->pos() - region_start_).manhattanLength() <= kClickSlopPx;
    // One rectangle, both consequences: the exact bounds are the processing
    // extent, and the index tiles it covers are the pass query. A click with
    // no drag clears both, which is also the clear-selection affordance the
    // tile rubber band never had.
    const bool had_tiles = !selected_tiles_.empty();
    if (was_click) {
      selected_tiles_.clear();
      if (cube_box_geo_) {
        cube_box_geo_.reset();
        emit cubeBoxCleared();
      }
      update();
      if (had_tiles) {
        emit tileSelectionChanged();
      }
      return;
    }
    const QPointF a = screenToMap(region_start_.x(), region_start_.y());
    const QPointF b = screenToMap(event->pos().x(), event->pos().y());
    std::set<std::size_t> hit;
    for (const auto idx : rectsInBox(index_tiles_, a.x(), a.y(), b.x(), b.y())) {
      hit.insert(idx);
    }
    const bool tiles_changed = hit != selected_tiles_;
    selected_tiles_ = std::move(hit);
    if (geo_mode_) {
      const auto [lat_a, lon_a] = canvasToGeo(a.x(), a.y());
      const auto [lat_b, lon_b] = canvasToGeo(b.x(), b.y());
      cube_box_geo_ = GeoRect{
        std::min(lat_a, lat_b), std::min(lon_a, lon_b),
        std::max(lat_a, lat_b), std::max(lon_a, lon_b)};
    }
    update();
    if (tiles_changed) {
      emit tileSelectionChanged();
    }
    if (cube_box_geo_) {
      emit cubeBoxSelected(
        cube_box_geo_->south, cube_box_geo_->west,
        cube_box_geo_->north, cube_box_geo_->east);
    }
    return;
  }
  if (!marking_) {return;}
  marking_ = false;
  const QPointF ca = screenToMap(mark_start_.x(), mark_start_.y());
  const QPointF cb = screenToMap(event->pos().x(), event->pos().y());
  update();
  const auto a = canvasToBag(ca.x(), ca.y());
  const auto b = canvasToBag(cb.x(), cb.y());
  if (!a || !b) {return;}   // bag not placeable: a mark would have no frame
  // Ignore a click with no drag (no extent).
  if (std::abs(a->x() - b->x()) < 1e-6 && std::abs(a->y() - b->y()) < 1e-6) {return;}
  emit boxMarked(QRectF(*a, *b).normalized());
}

}  // namespace marine_perception_tools
