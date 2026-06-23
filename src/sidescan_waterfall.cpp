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

#include "sidescan_waterfall.hpp"

#include <QColor>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QRectF>

#include <algorithm>
#include <cmath>
#include <vector>

namespace marine_perception_tools
{

SidescanWaterfall::SidescanWaterfall(QWidget * parent)
: QWidget(parent)
{
  setMinimumWidth(160);
  setAutoFillBackground(true);
}

void SidescanWaterfall::setImage(const QImage & image)
{
  image_ = image;
  update();
}

void SidescanWaterfall::setIndex(const WaterfallIndex & index)
{
  index_ = index;
}

void SidescanWaterfall::setContacts(const QVector<ContactMarker> & contacts)
{
  contacts_ = contacts;
  update();
}

void SidescanWaterfall::setMarkMode(bool on)
{
  mark_mode_ = on;
  setCursor(on ? Qt::CrossCursor : Qt::ArrowCursor);
}

bool SidescanWaterfall::widgetToMap(const QPoint & px, double & mx, double & my) const
{
  const int img_w = index_.pn + index_.sn;
  if (image_.isNull() || img_w <= 0 || index_.rows <= 0 || width() <= 0 || height() <= 0) {
    return false;
  }
  // Widget pixel -> image pixel (the image is stretched to fill the widget).
  const int col = std::clamp(
    static_cast<int>(static_cast<double>(px.x()) / width() * img_w), 0, img_w - 1);
  const int row = std::clamp(
    static_cast<int>(static_cast<double>(px.y()) / height() * index_.rows), 0,
    index_.rows - 1);

  const PingGeometry * geom = nullptr;
  std::size_t sample = 0;
  if (col < index_.pn) {
    if (row >= static_cast<int>(index_.port_geo.size())) {return false;}
    geom = &index_.port_geo[row];
    sample = static_cast<std::size_t>(index_.pn - 1 - col);  // near range at centre
  } else {
    if (row >= static_cast<int>(index_.stbd_geo.size())) {return false;}
    geom = &index_.stbd_geo[row];
    sample = static_cast<std::size_t>(col - index_.pn);
  }
  const GroundPoint gp = project_sample(*geom, sample);
  mx = gp.x;
  my = gp.y;
  return true;
}

void SidescanWaterfall::mousePressEvent(QMouseEvent * event)
{
  if (event->button() != Qt::LeftButton || !mark_mode_) {return;}
  marking_ = true;
  mark_start_ = event->pos();
  mark_cur_ = event->pos();
  update();
}

void SidescanWaterfall::mouseMoveEvent(QMouseEvent * event)
{
  if (!marking_) {return;}
  mark_cur_ = event->pos();
  update();
}

void SidescanWaterfall::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() != Qt::LeftButton || !marking_) {return;}
  marking_ = false;
  const QRect box = QRect(mark_start_, event->pos()).normalized();
  update();
  // Invert the four corners to map points; emit the map-space bounding box.
  const QPoint corners[4] = {
    box.topLeft(), box.topRight(), box.bottomLeft(), box.bottomRight()};
  bool have = false;
  double min_x = 0.0;
  double max_x = 0.0;
  double min_y = 0.0;
  double max_y = 0.0;
  for (const auto & c : corners) {
    double mx = 0.0;
    double my = 0.0;
    if (!widgetToMap(c, mx, my)) {continue;}
    if (!have) {
      min_x = max_x = mx;
      min_y = max_y = my;
      have = true;
    } else {
      min_x = std::min(min_x, mx);
      max_x = std::max(max_x, mx);
      min_y = std::min(min_y, my);
      max_y = std::max(max_y, my);
    }
  }
  if (!have) {return;}
  emit boxMarked(QRectF(QPointF(min_x, min_y), QPointF(max_x, max_y)).normalized());
}

void SidescanWaterfall::paintEvent(QPaintEvent * event)
{
  Q_UNUSED(event);
  QPainter painter(this);
  painter.fillRect(rect(), QColor(20, 24, 28));
  if (image_.isNull()) {
    painter.setPen(QColor(150, 160, 170));
    painter.drawText(rect(), Qt::AlignCenter, "waterfall");
    return;
  }
  // Stretch the slant-range image to fill the pane (across-track on x, scrub
  // distance on y). Smooth only vertically would be ideal; keep it simple.
  painter.drawImage(rect(), image_);

  // Box each contact at its closest-approach (apex) sample, with the id label. On a
  // slant-range waterfall a fixed target's echo traces a hyperbola; its apex (minimum
  // slant, the abeam ping) is where the target actually sits, so a box there reads
  // like the map box. We box once per continuous run of rows that ensonify the target,
  // so a target seen on two passes (e.g. after a turn) gets one box per pass.
  const int img_w = index_.pn + index_.sn;
  if (!contacts_.empty() && index_.rows > 0 && img_w > 0) {
    const double sx = static_cast<double>(width()) / img_w;
    const double sy = static_cast<double>(height()) / index_.rows;
    const QColor mark_color(255, 0, 255);   // magenta: reads on every palette

    // Data-array sample index of map point (cx,cy) on this ping (full slant range),
    // or -1 if the target is on the other channel's side / scale unknown.
    auto sample_at = [](const PingGeometry & g, double cx, double cy) -> int {
        if (g.metres_per_sample <= 0.0) {return -1;}
        const double dx = cx - g.sensor_x;
        const double dy = cy - g.sensor_y;
        const double along = dx * std::cos(g.yaw) + dy * std::sin(g.yaw);
        const double across =
          (dx * -std::sin(g.yaw) + dy * std::cos(g.yaw)) * g.lateral_sign;
        if (across <= 0.0) {return -1;}   // target is on the other channel's side
        const double alt = (g.altitude > 0.0) ? g.altitude : 0.0;
        const double horiz = std::sqrt(along * along + across * across);
        const double slant = std::sqrt(horiz * horiz + alt * alt);
        return static_cast<int>(slant / g.metres_per_sample - g.sample0 + 0.5);
      };

    for (const auto & c : contacts_) {
      const double footprint = 0.5 * std::max(static_cast<double>(c.w),
          static_cast<double>(c.h));

      // Walk a channel's rows; within each maximal run of ensonifying rows, box the
      // minimum-slant (closest) sample and label it.
      auto draw_passes = [&](const std::vector<PingGeometry> & geos, bool is_port) {
          const int max_i = is_port ? index_.pn : index_.sn;
          const int n = std::min(static_cast<int>(geos.size()), index_.rows);
          bool in_run = false;
          int best_i = 0;
          int best_row = -1;
          double best_mps = 0.0;
          auto flush = [&]() {
              if (best_row < 0) {return;}
              const int col = is_port ? (index_.pn - 1 - best_i) : (index_.pn + best_i);
              const double cxp = (col + 0.5) * sx;
              const double cyp = (best_row + 0.5) * sy;
              const double half_cols = (best_mps > 0.0) ? (footprint / best_mps) : 0.0;
              const double hw = std::max(6.0, half_cols * sx);
              const double hh = std::max(5.0, 3.0 * sy);
              QPen pen(mark_color);
              pen.setWidthF(2.0);
              painter.setPen(pen);
              painter.setBrush(Qt::NoBrush);
              painter.drawRect(QRectF(cxp - hw, cyp - hh, 2.0 * hw, 2.0 * hh));
              painter.drawText(QPointF(cxp + hw + 3.0, cyp - hh), c.id);
            };
          for (int r = 0; r < n; ++r) {
            const int i = sample_at(geos[r], c.x, c.y);
            const bool valid = (i >= 0 && i < max_i);
            if (valid) {
              if (!in_run || i < best_i) {
                best_i = i;
                best_row = r;
                best_mps = geos[r].metres_per_sample;
              }
              in_run = true;
            } else if (in_run) {
              flush();
              in_run = false;
              best_row = -1;
            }
          }
          if (in_run) {flush();}
        };

      draw_passes(index_.port_geo, true);
      draw_passes(index_.stbd_geo, false);
    }
  }

  // Rubber-band box while drawing a contact.
  if (marking_) {
    QPen pen(QColor(255, 0, 255));
    pen.setStyle(Qt::DashLine);
    painter.setPen(pen);
    painter.setBrush(QColor(255, 0, 255, 40));
    painter.drawRect(QRectF(mark_start_, mark_cur_).normalized());
  }
}

}  // namespace marine_perception_tools
