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

#ifndef SIDESCAN_WATERFALL_HPP_
#define SIDESCAN_WATERFALL_HPP_

#include <QImage>
#include <QPoint>
#include <QRectF>
#include <QWidget>

#include <vector>

#include "sidescan_geometry.hpp"

namespace marine_perception_tools
{

// Maps waterfall image pixels back to map coordinates so a box drawn on the
// waterfall can become a contact. Columns [0, pn) are the port channel (sample
// index pn-1-col), [pn, pn+sn) starboard (sample index col-pn); row r uses the
// r-th geometry (newest-at-top order, matching the image). project_sample() turns
// (geometry, sample index) into a map point.
struct WaterfallIndex
{
  int pn = 0;
  int sn = 0;
  int rows = 0;
  std::vector<PingGeometry> port_geo;   // size <= rows, image-row order
  std::vector<PingGeometry> stbd_geo;
};

// The classic uncorrected (slant-range) sidescan waterfall for the current scrub
// window: each row is a ping cycle, port samples on the left (far range outward)
// and starboard on the right, intensity = backscatter, newest ping at the top
// (matching the live rqt waterfall plugin). No georeferencing or slant-to-ground
// correction — the raw display analysts read. Stretched to fill the widget.
class SidescanWaterfall : public QWidget
{
  Q_OBJECT

public:
  explicit SidescanWaterfall(QWidget * parent = nullptr);

  void setImage(const QImage & image);
  void setIndex(const WaterfallIndex & index);

  // In mark mode, left-drag draws a contact box and emits boxMarked() (map coords).
  void setMarkMode(bool on);

signals:
  void boxMarked(const QRectF & map_rect);

protected:
  void paintEvent(QPaintEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;

private:
  // Map a widget pixel to a map point via the index. Returns false if unmappable.
  bool widgetToMap(const QPoint & px, double & mx, double & my) const;

  QImage image_;
  WaterfallIndex index_;
  bool mark_mode_ = false;
  bool marking_ = false;
  QPoint mark_start_;
  QPoint mark_cur_;
};

}  // namespace marine_perception_tools

#endif  // SIDESCAN_WATERFALL_HPP_
