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

#ifndef SIDESCAN_CANVAS_HPP_
#define SIDESCAN_CANVAS_HPP_

#include <QImage>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QWidget>

#include <optional>
#include <vector>

namespace marine_perception_tools
{

// A contact to draw on the map: centre + box extent (map metres) + label.
struct ContactMarker
{
  double x = 0.0;
  double y = 0.0;
  double w = 0.0;
  double h = 0.0;
  QString id;
};

// North-up 2D map view in the bizzy/map ENU plane (metres). Displays a rendered
// coverage image (painted swath) at its true map position, a boat track polyline,
// and a subtle measuring grid at a configurable spacing. Supports wheel zoom
// (about the cursor) and drag pan. Map east = +x (screen right), map north = +y
// (screen up), so the view is north-up.
class SidescanCanvas : public QWidget
{
  Q_OBJECT

public:
  explicit SidescanCanvas(QWidget * parent = nullptr);

  // Set the rendered coverage image and the map extent it covers. `origin_x/y` is
  // the south-west corner (min east, min north); the image is north-up (row 0 =
  // north edge). `res_m` is metres per pixel.
  void setCoverage(const QImage & image, double origin_x, double origin_y, double res_m);

  // Boat track as a polyline in map metres.
  void setTrack(const std::vector<QPointF> & track_map);

  void setGridSpacing(double metres);

  // Recenter the view on a map point (metres) without changing zoom — used to keep
  // the scrubbed window centred ("follow the playhead").
  void setCenter(double map_x, double map_y);

  // Fit the current coverage extent (or track) into the widget.
  void resetView();

  // In mark mode, left-drag draws a contact box (instead of panning) and emits
  // boxMarked() on release.
  void setMarkMode(bool on);

  // Contacts to overlay (map metres). Drawn wherever they fall in the current view.
  void setContacts(const QVector<ContactMarker> & contacts);

  // Transient cross-pane linked cursor at a map point (metres); nullopt clears it.
  void setCursorWorld(const std::optional<QPointF> & map_point);

signals:
  // A contact box was drawn, in map coordinates (metres).
  void boxMarked(const QRectF & map_rect);

  // Hovered map position (metres), for a cross-pane linked cursor (always valid).
  void hoverWorld(double map_x, double map_y, bool valid);

  // Middle-click map position (metres), for click-to-seek.
  void seekWorld(double map_x, double map_y);

protected:
  void paintEvent(QPaintEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;

private:
  QPointF mapToScreen(double mx, double my) const;
  QPointF screenToMap(double sx, double sy) const;
  void drawGrid(QPainter & painter) const;

  QImage coverage_;
  double cov_origin_x_ = 0.0;   // SW corner east (m)
  double cov_origin_y_ = 0.0;   // SW corner north (m)
  double cov_res_m_ = 1.0;      // metres per coverage pixel
  bool have_coverage_ = false;

  std::vector<QPointF> track_;

  double grid_spacing_m_ = 10.0;
  double px_per_m_ = 4.0;       // zoom
  QPointF center_map_{0.0, 0.0};  // map point shown at the widget centre

  QPoint last_drag_pos_;

  bool mark_mode_ = false;
  bool marking_ = false;        // mid box-drag
  QPoint mark_start_;           // screen px
  QPoint mark_cur_;             // screen px
  QVector<ContactMarker> contacts_;
  std::optional<QPointF> cursor_world_;   // cross-pane linked cursor (map metres)
};

}  // namespace marine_perception_tools

#endif  // SIDESCAN_CANVAS_HPP_
