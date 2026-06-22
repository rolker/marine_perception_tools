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
}

}  // namespace marine_perception_tools
