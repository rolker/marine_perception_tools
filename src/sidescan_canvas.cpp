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
#include <QWheelEvent>

#include <algorithm>
#include <cmath>

namespace marine_perception_tools
{

SidescanCanvas::SidescanCanvas(QWidget * parent)
: QWidget(parent)
{
  setMinimumSize(480, 360);
  setMouseTracking(false);
  setAutoFillBackground(true);
}

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
  center_map_ = QPointF(map_x, map_y);
  update();
}

void SidescanCanvas::setMarkMode(bool on)
{
  mark_mode_ = on;
  setCursor(on ? Qt::CrossCursor : Qt::ArrowCursor);
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
  if (have_coverage_) {
    min_x = cov_origin_x_;
    min_y = cov_origin_y_;
    max_x = cov_origin_x_ + coverage_.width() * cov_res_m_;
    max_y = cov_origin_y_ + coverage_.height() * cov_res_m_;
    have = true;
  } else if (!track_.empty()) {
    min_x = max_x = track_.front().x();
    min_y = max_y = track_.front().y();
    for (const auto & p : track_) {
      min_x = std::min(min_x, p.x());
      max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y());
      max_y = std::max(max_y, p.y());
    }
    have = true;
  }
  if (!have) {return;}
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

void SidescanCanvas::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  QPainter painter(this);
  painter.fillRect(rect(), QColor(20, 24, 28));

  if (have_coverage_) {
    // Image extent in map: SW corner (origin) to NE corner. The image is north-up
    // (row 0 = north), so its top-left in map is (origin_x, origin_y + h*res).
    const double w_m = coverage_.width() * cov_res_m_;
    const double h_m = coverage_.height() * cov_res_m_;
    const QPointF nw = mapToScreen(cov_origin_x_, cov_origin_y_ + h_m);
    const QRectF dst(nw, QSizeF(w_m * px_per_m_, h_m * px_per_m_));
    painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
    painter.drawImage(dst, coverage_);
  }

  drawGrid(painter);

  if (track_.size() > 1) {
    QPen pen(QColor(120, 200, 255, 180));
    pen.setWidthF(1.5);
    painter.setPen(pen);
    QPolygonF poly;
    poly.reserve(static_cast<int>(track_.size()));
    for (const auto & p : track_) {
      poly << mapToScreen(p.x(), p.y());
    }
    painter.drawPolyline(poly);
  }

  // Contact markers (map-anchored), drawn wherever they fall in the current view.
  if (!contacts_.empty()) {
    QPen pen(QColor(255, 210, 60));
    pen.setWidthF(1.5);
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    for (const auto & c : contacts_) {
      const QPointF nw = mapToScreen(c.x - 0.5 * c.w, c.y + 0.5 * c.h);
      const double wpx = std::max(6.0, c.w * px_per_m_);
      const double hpx = std::max(6.0, c.h * px_per_m_);
      const QRectF r(nw, QSizeF(wpx, hpx));
      painter.drawRect(r);
      if (!c.id.isEmpty()) {
        painter.drawText(QPointF(r.left(), r.top() - 2), c.id);
      }
    }
  }

  // Rubber-band box while drawing a contact.
  if (marking_) {
    QPen pen(QColor(255, 210, 60));
    pen.setStyle(Qt::DashLine);
    painter.setPen(pen);
    painter.setBrush(QColor(255, 210, 60, 40));
    painter.drawRect(QRectF(mark_start_, mark_cur_).normalized());
  }
}

void SidescanCanvas::wheelEvent(QWheelEvent * event)
{
  const double steps = event->angleDelta().y() / 120.0;
  if (steps == 0.0) {return;}
  const QPointF cursor = event->position();
  const QPointF before = screenToMap(cursor.x(), cursor.y());
  const double factor = std::pow(1.2, steps);
  px_per_m_ = std::clamp(px_per_m_ * factor, 0.05, 500.0);
  // Keep the map point under the cursor fixed.
  const QPointF after = screenToMap(cursor.x(), cursor.y());
  center_map_ += before - after;
  update();
}

void SidescanCanvas::mousePressEvent(QMouseEvent * event)
{
  if (event->button() != Qt::LeftButton) {return;}
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
  if (!(event->buttons() & Qt::LeftButton)) {return;}
  if (marking_) {
    mark_cur_ = event->pos();
    update();
    return;
  }
  const QPoint delta = event->pos() - last_drag_pos_;
  last_drag_pos_ = event->pos();
  center_map_ += QPointF(-delta.x() / px_per_m_, delta.y() / px_per_m_);
  update();
}

void SidescanCanvas::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() != Qt::LeftButton || !marking_) {return;}
  marking_ = false;
  const QPointF a = screenToMap(mark_start_.x(), mark_start_.y());
  const QPointF b = screenToMap(event->pos().x(), event->pos().y());
  update();
  // Ignore a click with no drag (no extent).
  if (std::abs(a.x() - b.x()) < 1e-6 && std::abs(a.y() - b.y()) < 1e-6) {return;}
  emit boxMarked(QRectF(a, b).normalized());
}

}  // namespace marine_perception_tools
