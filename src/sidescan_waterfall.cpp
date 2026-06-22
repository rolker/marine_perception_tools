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
#include <QPainter>

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
