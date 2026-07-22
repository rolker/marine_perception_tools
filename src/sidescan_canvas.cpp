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
#include <QFont>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <QTransform>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace marine_perception_tools
{

namespace
{

// Local-equirectangular metres per degree of latitude (mean Earth). The same
// constant the bridge's point query uses; display-grade, not geodetic.
constexpr double kMetersPerDegLat = 111320.0;

// Nav-track arrowheads roughly this many screen pixels apart.
constexpr double kArrowSpacingPx = 30.0;

}  // namespace

SidescanCanvas::SidescanCanvas(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(480, 360);
  setMouseTracking(true);   // hover reporting (hoverWorld) needs moves without a button
  setAutoFillBackground(true);
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
  rebuildGeoLayerGeometry();
  update();
}

void SidescanCanvas::setNavTrack(
  std::vector<std::vector<std::pair<double, double>>> segments)
{
  nav_segments_geo_ = std::move(segments);
  rebuildGeoLayerGeometry();
  update();
}

void SidescanCanvas::setIndexTiles(const std::vector<GeoRect> & tiles)
{
  index_tiles_geo_ = tiles;
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

void SidescanCanvas::fitGeo(double south, double west, double north, double east)
{
  fit_south_ = south;
  fit_west_ = west;
  fit_north_ = north;
  fit_east_ = east;
  fit_pending_ = true;
  user_adjusted_ = false;
  update();
}

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
    update();
  }
}

void SidescanCanvas::setCenter(double map_x, double map_y)
{
  if (!mapPlaceable()) {
    return;
  }
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

void SidescanCanvas::drawNavTrack(QPainter & painter) const
{
  if (nav_segments_.empty()) {
    return;
  }
  QPen pen(QColor(255, 255, 255, 153));   // white, 60 % alpha
  pen.setWidthF(1.2);
  painter.setPen(pen);
  painter.setBrush(QColor(255, 255, 255, 153));
  for (const auto & seg : nav_segments_) {
    if (seg.size() < 2) {
      continue;
    }
    QPolygonF screen;
    screen.reserve(seg.size());
    for (const auto & p : seg) {
      screen << mapToScreen(p.x(), p.y());
    }
    painter.setBrush(Qt::NoBrush);
    painter.drawPolyline(screen);
    // Direction arrowheads (older -> newer) roughly every kArrowSpacingPx.
    painter.setBrush(QColor(255, 255, 255, 153));
    double since_last = kArrowSpacingPx;   // arrow near the segment start too
    for (int i = 1; i < screen.size(); ++i) {
      const QPointF d = screen[i] - screen[i - 1];
      const double len = std::hypot(d.x(), d.y());
      since_last += len;
      if (since_last < kArrowSpacingPx || len < 1e-9) {
        continue;
      }
      since_last = 0.0;
      const QPointF mid = (screen[i - 1] + screen[i]) * 0.5;
      const QPointF dir(d.x() / len, d.y() / len);
      const QPointF ortho(-dir.y(), dir.x());
      constexpr double kA = 5.0;   // arrow size, px
      QPolygonF arrow;
      arrow << (mid + dir * kA)
            << (mid - dir * kA * 0.6 + ortho * kA * 0.6)
            << (mid - dir * kA * 0.6 - ortho * kA * 0.6);
      painter.drawPolygon(arrow);
    }
  }
}

void SidescanCanvas::drawIndexTiles(QPainter & painter) const
{
  if (index_tiles_.empty()) {
    return;
  }
  const QRectF viewport(0, 0, width(), height());
  QPen outline(QColor(0, 200, 255, 60));
  outline.setWidthF(1.0);
  for (std::size_t i = 0; i < index_tiles_.size(); ++i) {
    const auto & t = index_tiles_[i];
    // Canvas y grows north, screen y grows down: NW corner is (x0, y1).
    const QPointF nw = mapToScreen(t.x0, t.y1);
    const QPointF se = mapToScreen(t.x1, t.y0);
    const QRectF r(nw, se);
    if (!r.intersects(viewport)) {
      continue;
    }
    const bool selected = selected_tiles_.count(i) > 0;
    painter.setPen(selected ? QPen(QColor(0, 255, 255, 220), 1.5) : outline);
    painter.setBrush(selected ? QColor(0, 255, 255, 70) : Qt::NoBrush);
    painter.drawRect(r);
  }
  painter.setBrush(Qt::NoBrush);
}

void SidescanCanvas::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  QPainter painter(this);
  painter.fillRect(rect(), QColor(20, 24, 28));
  applyPendingFit();

  // Survey layers (geo mode), bottom-up: store basemap, nav track, tile grid.
  if (geo_mode_ && !store_tile_rects_.empty()) {
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    for (std::size_t i = 0; i < store_tile_rects_.size(); ++i) {
      const auto & r = store_tile_rects_[i];
      const QPointF nw = mapToScreen(r.left(), r.top() + r.height());
      const QPointF se = mapToScreen(r.left() + r.width(), r.top());
      const QRectF target(nw, se);
      if (!target.intersects(rect())) {
        continue;
      }
      painter.drawImage(target, store_tiles_[i].image);
    }
    painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
  }

  drawGrid(painter);
  if (geo_mode_) {
    drawNavTrack(painter);
    drawIndexTiles(painter);
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

  // Rubber band while selecting tiles (ctrl-drag).
  if (band_selecting_) {
    QPen pen(QColor(0, 255, 255));
    pen.setStyle(Qt::DashLine);
    painter.setPen(pen);
    painter.setBrush(QColor(0, 255, 255, 30));
    painter.drawRect(QRectF(band_start_, band_cur_).normalized());
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
}

void SidescanCanvas::wheelEvent(QWheelEvent * event)
{
  const double steps = event->angleDelta().y() / 120.0;
  if (steps == 0.0) {return;}
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
}

void SidescanCanvas::mousePressEvent(QMouseEvent * event)
{
  if (event->button() == Qt::MiddleButton) {
    const QPointF m = screenToMap(event->pos().x(), event->pos().y());
    const auto bag = canvasToBag(m.x(), m.y());
    if (bag) {
      emit seekWorld(bag->x(), bag->y());   // middle-click: seek to this map position
    }
    return;
  }
  if (event->button() != Qt::LeftButton) {return;}
  if ((event->modifiers() & Qt::ControlModifier) && !index_tiles_.empty()) {
    band_selecting_ = true;
    band_start_ = event->pos();
    band_cur_ = event->pos();
    update();
    return;
  }
  if (mark_mode_) {
    marking_ = true;
    mark_start_ = event->pos();
    mark_cur_ = event->pos();
    update();
  } else {
    last_drag_pos_ = event->pos();
  }
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
  if (!(event->buttons() & Qt::LeftButton)) {return;}
  if (band_selecting_) {
    band_cur_ = event->pos();
    update();
    return;
  }
  if (marking_) {
    mark_cur_ = event->pos();
    update();
    return;
  }
  const QPoint delta = event->pos() - last_drag_pos_;
  last_drag_pos_ = event->pos();
  center_map_ += QPointF(-delta.x() / px_per_m_, delta.y() / px_per_m_);
  user_adjusted_ = true;
  fit_pending_ = false;
  update();
}

void SidescanCanvas::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() != Qt::LeftButton) {return;}
  if (band_selecting_) {
    band_selecting_ = false;
    bool changed = false;
    if ((event->pos() - band_start_).manhattanLength() <= 4) {
      // A ctrl-click, not a drag: toggle the tile under the cursor.
      const QPointF m = screenToMap(event->pos().x(), event->pos().y());
      const int hit = hitRect(index_tiles_, m.x(), m.y());
      if (hit >= 0) {
        toggleSelection(selected_tiles_, static_cast<std::size_t>(hit));
        changed = true;
      }
    } else {
      // Rubber band: add every intersecting tile to the selection.
      const QPointF a = screenToMap(band_start_.x(), band_start_.y());
      const QPointF b = screenToMap(event->pos().x(), event->pos().y());
      for (const auto idx : rectsInBox(index_tiles_, a.x(), a.y(), b.x(), b.y())) {
        changed = selected_tiles_.insert(idx).second || changed;
      }
    }
    update();
    if (changed) {
      emit tileSelectionChanged();
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
